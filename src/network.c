
#include "buttons.h"
#include "log.h"
#include "modbus.h"
#include <cJSON.h>
#include <hardware/flash.h>
#include <hardware/irq.h>
#include <hardware/sync.h>
#include <hardware/watchdog.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <pico/sync.h>
#include <pico/util/queue.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cli.h"

#include "MQTTClient.h"
#include "dali.h"
#include "dhcp.h"
#include "dns.h"
#include "mqtt_interface.h"
#include "network.h"
#include "pico/time.h"
#include "socket.h"
#include "w5100s.h"
#include "w5x00_spi.h"
#include <pico/unique_id.h>
#include <wizchip_conf.h>

#define ETHERNET_BUF_MAX_SIZE (1024 * 2)

/* Socket */
#define SOCKET_DHCP 0
#define SOCKET_DNS 1
#define SOCKET_MQTT 2

/* Retry count */
#define DHCP_RETRY_COUNT 5
#define DNS_RETRY_COUNT 5

#define PORT_MQTT 1883

/* Timeout */
#define DEFAULT_TIMEOUT 1000 // 1 second

typedef struct {
  device_event_type_t type;
  void *data;
} device_action_t;

static queue_t updatesQueue;

static char switchDeviceID[17];

#define MQTT_KEEP_ALIVE 60

static wiz_NetInfo g_net_info = {
    .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x57}, // MAC address
    .ip = {192, 168, 1, 36},                     // IP address
    .sn = {255, 255, 255, 0},                    // Subnet Mask
    .gw = {192, 168, 1, 1},                      // Gateway
    .dns = {192, 168, 1, 1},                     // DNS server
    .dhcp = NETINFO_DHCP                         // DHCP enable/disable
};

/* DHCP */
static uint8_t dhcpBuf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};

/* DNS */
static uint8_t mqtt_ip[4] = {
    0,
};
static uint8_t dnsBuf[ETHERNET_BUF_MAX_SIZE] = {
    0,
}; // common buffer

/* MQTT */
char msg[ETHERNET_BUF_MAX_SIZE] = {
    0,
};
char topic[100];

static volatile uint16_t g_msec_cnt = 0;

static Network g_mqtt_network;
static MQTTClient g_mqtt_client;
static MQTTPacket_connectData g_mqtt_packet_connect_data = MQTTPacket_connectData_initializer;
static MQTTMessage g_mqtt_message;

static uint8_t g_mqtt_send_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};
static uint8_t g_mqtt_recv_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};

static struct repeating_timer g_timer;

void send_button_state(int fixture_id, int button_id, bool pressed);

unsigned long millis() { return to_ms_since_boot(get_absolute_time()); }

static bool repeating_timer_callback(struct repeating_timer *t);
static void wizchip_dhcp_init(void);
static void wizchip_dhcp_assign(void);
static void wizchip_dhcp_conflict(void);

typedef struct {
  char *topic;
  void (*handler)(MessageData *);
} subscritption_t;

static void processHAStatusMessage(MessageData *msg);
static void processUpdateBinding(MessageData *msg);
static void processUpdateModbus(MessageData *msg);
static void processUpdateDALI(MessageData *msg);
static void send_modbus_state(int device_id, int coil_id, bool is_on);
static void send_button_binding_state(int fixture_id, int button_id, uint32_t bindingval);

static char modbusSubTopic[64];
static char bindingSubTopic[64];
static char daliSubTopic[64];

static bool daliPublishDevicesPending = false;

static uint8_t dhcp_status;
static bool device_announcement_needed;

static const subscritption_t subscription_topics[] = {
    /* Switch Bindings */ {.topic = bindingSubTopic, .handler = processUpdateBinding},
    /* Modbus Coil Set */ {.topic = modbusSubTopic, .handler = processUpdateModbus},
    /* DALI Value Change */ {.topic = daliSubTopic, .handler = processUpdateDALI},
    /* Home Assistant Status Change */ {.topic = "homeassistant/status", .handler = processHAStatusMessage}};

void enqueue_device_update(device_event_type_t action, void *data) {
  device_action_t evt = {
      .type = action,
      .data = data,
  };

  if (!queue_try_add(&updatesQueue, &evt)) {
    printf("Update queue full");
  }
}

static void process_event_queue() {
  device_action_t evt;
  button_ctx_t *btn;
  dali_dev_data_t *daliDev;
  int retval, idx, device, address;

  if (queue_try_remove(&updatesQueue, &evt)) {
    switch (evt.type) {
    case EVT_BTN_RELEASED:
    case EVT_BTN_HELD:
    case EVT_BTN_PRESSED:
      btn = evt.data;
      idx = btn - button_ctx;
      device = idx / NUM_BUTTONS_PER_FIXTURE;
      address = idx % NUM_BUTTONS_PER_FIXTURE;

      g_mqtt_message.payloadlen = sprintf(msg,
                                          "{\"bus\": \"button\", \"device\": %d, \"address\": %d, "
                                          "\"event_type\": \"%s\"}",
                                          device, address,
                                          evt.type == EVT_BTN_PRESSED    ? "press"
                                          : evt.type == EVT_BTN_RELEASED ? "release"
                                                                         : "hold");

      sprintf(topic, "switchy/%s/button/%d/%d", switchDeviceID, device, address);
      printf("sending %.*s (%d) to %s\n", g_mqtt_message.payloadlen, (char *)g_mqtt_message.payload, g_mqtt_message.payloadlen, topic);
      retval = MQTTPublish(&g_mqtt_client, topic, &g_mqtt_message);

      if (retval < 0) {
        printf("evt send Publish failed : %d\n", retval);
      }
      break;

    case EVT_MODBUS_DEVICE_DISCOVERED:
      address = (int)evt.data;
      g_mqtt_message.payloadlen =  modbus_write_discovery_message(1, address, (char *) g_mqtt_message.payload, sizeof(msg), topic, sizeof(topic), switchDeviceID);
      retval = MQTTPublish(&g_mqtt_client, topic, &g_mqtt_message);
      if (retval < 0) {
        printf("evt send Publish failed : %d\n", retval);
      }
      break;

    case EVT_MODBUS_COIL_STATE_CHANGED:
      device = 1;
      address = (int)evt.data;
      send_modbus_state(device, address, modbus_get_coil(device, address));
      break;

    case EVT_BUTTON_BINDING_CHANGED:
      btn = evt.data;
      idx = btn - button_ctx;
      device = idx / NUM_BUTTONS_PER_FIXTURE;
      address = idx % NUM_BUTTONS_PER_FIXTURE;

      send_button_binding_state(device, address, get_binding(device, address));
      break;

    case EVT_DALI_LEVEL_CHANGED:
      daliDev = evt.data;
      break;

    case EVT_DALI_DEVICE_SCAN_COMPLETED:
      printf("DALI bus scan completed.\n");
      daliPublishDevicesPending = true;
      break;
    }
  }
}

void send_button_state(int fixture_id, int button_id, bool pressed) {
  sprintf(msg, pressed ? "ON" : "OFF");
  sprintf(topic, "switchy/%s/button/%d/%d", switchDeviceID, fixture_id, button_id);
  g_mqtt_message.payloadlen = strlen(msg);
  int retval = MQTTPublish(&g_mqtt_client, topic, &g_mqtt_message);

  if (retval < 0) {
    printf("sending button state Publish failed : %d\n", retval);
    while (1)
      ;
  }
}

static void send_button_binding_state(int fixture_id, int button_id, uint32_t bindingval) {
  binding_t binding;
  decode_binding(bindingval, &binding);
  int len;

  switch (binding.type) {
  case BINDING_TYPE_MODBUS:
    len = sprintf(msg, "modbus%02d%02d", binding.device, binding.address);
    break;
  case BINDING_TYPE_DALI:
    len = sprintf(msg, "dali%02d%02d", binding.device, binding.address);
    break;

  default:
    printf("Illegal binding type %d.  Treating it as none\n", binding.type);
    // fall througth
  case BINDING_TYPE_NONE:
    len = sprintf(msg, "none");
    break;
  }

  sprintf(topic, "switchy/%s/binding/%d/%d", switchDeviceID, fixture_id, button_id);
  g_mqtt_message.payloadlen = len;
  int retval = MQTTPublish(&g_mqtt_client, topic, &g_mqtt_message);

  if (retval < 0) {
    printf("sending binding state Publish failed : %d\n", retval);
    while (1)
      ;
  }
}

static void send_modbus_state(int modbus_device_id, int coil_id, bool is_on) {
  sprintf(msg, is_on ? "ON" : "OFF");
  sprintf(topic, "switchy/%s/modbus/%d/%d", switchDeviceID, modbus_device_id, coil_id);
  g_mqtt_message.payloadlen = strlen(msg);
  int retval = MQTTPublish(&g_mqtt_client, topic, &g_mqtt_message);
  if (retval < 0) {
    printf("sending modbus state Publish failed : %d\n", retval);
    while (1)
      ;
  }
}


static void send_hub_ha_discovery() {
  // Send discovery message for the action binding.
  g_mqtt_message.payloadlen += sprintf(msg,
                                       "{"
                                       "\"device\": {"
                                       " \"identifiers\": [ \"%s\" ]"
                                       ",\"serial_number\": \"%s\""
                                       ",\"name\": \"Switchy\""
                                       ",\"manufacturer\": \"Mechination\""
                                       ",\"model\": \"Switch Hub\""
                                       ",\"sw_version\": \"1.0.1\""
                                       ",\"hw_version\": \"1.0.0\""
                                       "}"
                                       ",\"origin\": { \"name\": \"embedded\"}"
                                       ",\"components\": {"
                                       " \"long_press_initial_delay\": {\"platform\": \"number\","
                                       " \"name\": \"Long Press Initial Delay\","
                                       " \"min\": 0,"
                                       " \"max\": 10000,"
                                       " \"mode\": \"box\","
                                       " \"entity_category\": \"config\","
                                       " \"unit_of_measurement\": \"ms\","
                                       " \"unique_id\": \"%s_long_press_initial_delay\","
                                       " \"command_topic\": \"switchy/%s/config/long_press_initial_delay/set\","
                                       " \"state_topic\":\"switchy/%s/config/long_press_initial_delay\""
                                       "}"
                                       "}"
                                       "}",
                                       switchDeviceID, switchDeviceID, switchDeviceID, switchDeviceID, switchDeviceID);
  sprintf(topic, "homeassistant/device/%s/config", switchDeviceID);

  printf("Sending %.*s (%d bytes) to %s\n", g_mqtt_message.payloadlen, msg, g_mqtt_message.payloadlen, topic);

  int retval = MQTTPublish(&g_mqtt_client, topic, &g_mqtt_message);

  if (retval != 0) {
    printf("Sending Hub Discovery failed : %d\n", retval);
    while (1)
      ;
  }
}

static void send_button_ha_discovery(int fixture_id) {
  // Send discovery message for the action binding.

  char *p = msg;
  p += sprintf(p,
               "{"
               "\"device\": {"
               " \"identifiers\": [ \"%s_fixture%d\" ]"
               ",\"via_device\": \"%s\""
               ",\"name\": \"Fixture %02d\""
               "}"
               ",\"o\": { \"name\": \"embedded\"}"
               ",\"components\": {",
               switchDeviceID, fixture_id, switchDeviceID, fixture_id);

  for (int button_id = 0; button_id < NUM_BUTTONS_PER_FIXTURE; button_id++) {
    p = msg;
    p += sprintf(p, "{\"unique_id\": \"%s_button_fixture%d_binding%d\"", switchDeviceID, fixture_id, button_id);
    p += sprintf(p, ",\"name\": \"Button %d Binding\"", button_id);

    p += sprintf(p, ",\"object_id\": \"button_fixture%d_binding%d\"", fixture_id, button_id);
    p += sprintf(p, ",\"state_topic\": \"switchy/%s/binding/%d/%d\"", switchDeviceID, fixture_id, button_id);
    p += sprintf(p, ",\"command_topic\": \"switchy/%s/binding/%d/%d/set\"", switchDeviceID, fixture_id, button_id);
    p += sprintf(p, ",\"entity_category\": \"config\"");
    p += sprintf(p, ",\"availability\": [{\"topic\": \"switchy/%s/available\"}]", switchDeviceID);
    p += sprintf(p, ",\"optimistic\": false");
    p += sprintf(p, ",\"pattern\": \"(none|modbus\\\\d{4}|dali\\\\d{4})\"");

    p += sprintf(p, ",\"qos\": 0");
    p += sprintf(p,
                 ",\"device\": { \"identifiers\": [ \"%s_fixture%d\"]"
                 ",\"via_device\": \"%s\""
                 ",\"name\": \"Button Fixture %02d\"}}",
                 switchDeviceID, fixture_id, switchDeviceID, fixture_id);

    sprintf(topic, "homeassistant/text/%s/button_fixture%d_binding%d/config", switchDeviceID, fixture_id, button_id);

    g_mqtt_message.payloadlen = p - msg;
    // printf("BTN Sending %.*s (%d bytes) to %s\n", g_mqtt_message.payloadlen, (char *)g_mqtt_message.payload, g_mqtt_message.payloadlen, topic);
    int retval = MQTTPublish(&g_mqtt_client, topic, &g_mqtt_message);

    if (retval != 0) {
      printf("Sending Button Discovery failed : %d\n", retval);
      while (1)
        ;
    }

    // Also send button events
    p = msg;
    p += sprintf(p, "{\"unique_id\": \"%s_button_%d_button%d\"", switchDeviceID, fixture_id, button_id);
    p += sprintf(p, ",\"name\": \"Button %d\"", button_id);

    p += sprintf(p, ",\"object_id\": \"button_%d_%d\"", fixture_id, button_id);
    p += sprintf(p, ",\"state_topic\": \"switchy/%s/button/%d/%d\"", switchDeviceID, fixture_id, button_id);
    p += sprintf(p, ",\"availability\": [{\"topic\": \"switchy/%s/available\"}]", switchDeviceID);
    p += sprintf(p, ",\"event_types\": [\"press\", \"release\", \"hold\"]");

    p += sprintf(p, ",\"qos\": 0");
    p += sprintf(p,
                 ",\"device\": { \"identifiers\": [ \"%s_fixture%d\"]"
                 ",\"via_device\": \"%s\""
                 ",\"name\": \"Button Fixture %02d\"}}",
                 switchDeviceID, fixture_id, switchDeviceID, fixture_id);

    sprintf(topic, "homeassistant/event/%s/button_%d_%d/config", switchDeviceID, fixture_id, button_id);

    g_mqtt_message.payloadlen = p - msg;
    // printf("BTN Sending %.*s (%d bytes) to %s\n", g_mqtt_message.payloadlen, (char *)g_mqtt_message.payload, g_mqtt_message.payloadlen, topic);
    retval = MQTTPublish(&g_mqtt_client, topic, &g_mqtt_message);

    if (retval != 0) {
      printf("Sending Button Discovery failed : %d\n", retval);
      while (1)
        ;
    }
  }
}

static void send_available(bool is_available) {
  sprintf(msg, is_available ? "online" : "offline");
  sprintf(topic, "switchy/%s/available", switchDeviceID);
  g_mqtt_message.payloadlen = strlen(msg);
  int retval = MQTTPublish(&g_mqtt_client, topic, &g_mqtt_message);

  if (retval != 0) {
    printf("Sending availabiilty Publish failed : %d\n", retval);
    while (1)
      ;
  }
}

void enumerate_all() {
  // Start tasks to enumerate the different busses.
  // dali_enumerate();
  modbus_enumerate();
  buttons_enumerate();
}

void network_init() {
  pico_get_unique_board_id_string(switchDeviceID, sizeof(switchDeviceID));
  sprintf(bindingSubTopic, "switchy/%s/binding/+/+/set", switchDeviceID);
  sprintf(modbusSubTopic, "switchy/%s/modbus/+/+/set", switchDeviceID);
  sprintf(daliSubTopic, "switchy/%s/dali/+/set", switchDeviceID);
  // This is a big queue for receiving updates from the main device loop.
  queue_init(&updatesQueue, sizeof(device_action_t), 512);

  cli_init();
}

void network_thread() {

  wizchip_spi_initialize();
  wizchip_cris_initialize();

  wizchip_reset();
  wizchip_initialize();
  wizchip_check();

  network_initialize(g_net_info);

  // Create a separate alarm pool to operate off core 1.
  alarm_pool_t *core1pool = alarm_pool_create(2, 16);
  alarm_pool_add_repeating_timer_us(core1pool, -1000, repeating_timer_callback, NULL, &g_timer);

  wizchip_dhcp_init();
  DNS_init(SOCKET_DNS, dnsBuf);
  NewNetwork(&g_mqtt_network, SOCKET_MQTT);

  // if (retval != 0) {
  //   printf(" MQTT connect failed : %d\n", retval);

  //   while (1)
  //     ;
  // }

  g_mqtt_message.qos = QOS0;
  g_mqtt_message.retained = 0;
  g_mqtt_message.dup = 0;
  g_mqtt_message.payload = msg;
  g_mqtt_message.payloadlen = 0;

  int retval;

  for (;;) {
    dhcp_status = DHCP_run();

    switch (dhcp_status) {
    case DHCP_IP_ASSIGN:
    case DHCP_IP_CHANGED:
    case DHCP_IP_LEASED:

      // We have a valid IP address.  Now see if we have valid DNS
      if (mqtt_ip[0] == 0) {
        printf("Blocking DNS request for %s\n", MQTT_SERVER);
        if (DNS_run(g_net_info.dns, (uint8_t *) MQTT_SERVER, mqtt_ip)) {
          printf("IP of mqtt server is %d.%d.%d.%d\n", mqtt_ip[0], mqtt_ip[1], mqtt_ip[2], mqtt_ip[3]);
        } else {
          printf("Could not obtain IP address of MQTT server\n");
          while (1) {
            ;
          }
        }
      } else {
        // We have a valid
        int connectState = getSn_SR(g_mqtt_network.my_socket);

        switch (connectState) {
        case SOCK_CLOSED:
          printf("Socket closed.  Attempting to connect\n");
          if (ConnectNetwork(&g_mqtt_network, mqtt_ip, PORT_MQTT)) {
            printf("Socket connected\n");
            MQTTClientInit(&g_mqtt_client, &g_mqtt_network, DEFAULT_TIMEOUT, g_mqtt_send_buf, ETHERNET_BUF_MAX_SIZE, g_mqtt_recv_buf, ETHERNET_BUF_MAX_SIZE);
            g_mqtt_packet_connect_data.MQTTVersion = 3;
            g_mqtt_packet_connect_data.cleansession = 1;
            g_mqtt_packet_connect_data.willFlag = 0;
            g_mqtt_packet_connect_data.keepAliveInterval = MQTT_KEEP_ALIVE;
            g_mqtt_packet_connect_data.clientID.cstring = switchDeviceID;
            g_mqtt_packet_connect_data.username.cstring = MQTT_USERNAME;
            g_mqtt_packet_connect_data.password.cstring = MQTT_PASSWORD;

            printf("MQTT Handshake\n");
            retval = MQTTConnect(&g_mqtt_client, &g_mqtt_packet_connect_data);

            if (retval == 0) {
              subscritption_t *s = (subscritption_t *)subscription_topics;
              for (int i = 0; i < sizeof(subscription_topics) / sizeof(subscription_topics[0]); i++, s++) {
                printf("Subscribing to %s\n", s->topic);
                retval = MQTTSubscribe(&g_mqtt_client, s->topic, QOS0, s->handler);
                if (retval) {
                  printf("Could not subscribe to topic %s\n", topic);
                  while (1)
                    ;
                }
              }

              send_hub_ha_discovery();

              // Print MQTT messages for each button's bindings.
              for (int fixture_id = 0; fixture_id < NUM_FIXTURES; fixture_id++) {
                send_button_ha_discovery(fixture_id);
                for (int button_id = 0; button_id < NUM_BUTTONS_PER_FIXTURE; button_id++) {
                  send_button_binding_state(fixture_id, button_id, get_binding(fixture_id, button_id));
                  send_button_state(fixture_id, button_id, is_button_pressed(fixture_id, button_id));
                }
              }

              enumerate_all();

              // Send device online message
              send_available(true);
            } else {
              printf("Could not complete MQTT handshake\n");
              disconnect(g_mqtt_network.my_socket);
            }
          } else {
            printf("Could not connect to MQTT Server\n");
          }
          break;

        case SOCK_ESTABLISHED:
          process_event_queue();
          if (daliPublishDevicesPending) {
            printf("Publishing DALI devices\n");

            for (int i = 0; i < 64; i++) {
              g_mqtt_message.payloadlen = dali_write_discovery_msg(&dali_devices[i], msg, sizeof(msg), topic, sizeof(topic), switchDeviceID);
              int retval = MQTTPublish(&g_mqtt_client, topic, &g_mqtt_message);

              if (retval != 0) {
                printf("Sending Dali Device Discovery %d failed: %d\n", i, retval);
              }
            }
            // send_dali_ha_discovery();
            daliPublishDevicesPending = false;
          }

          for (int i = 0; i < 64; i++) {
            if (dali_devices[i].values_changed) {
              dali_devices[i].values_changed = false;
              g_mqtt_message.payloadlen = dali_write_values_msg(&dali_devices[i], msg, sizeof(msg), topic, sizeof(topic), switchDeviceID);
              int retval = MQTTPublish(&g_mqtt_client, topic, &g_mqtt_message);

              if (retval != 0) {
                printf("Sending Dali Device Discovery %d failed: %d\n", i, retval);
              }
            }
          }

          if ((retval = MQTTYield(&g_mqtt_client, 50)) < 0) {
            printf(" MQTT Yield error : %d\n", retval);
            disconnect(g_mqtt_network.my_socket);
          }
          break;

        default:
          printf("Socket state 0x%02x\r", connectState);
          break;
        }
      }

      break;
    case DHCP_FAILED:
    case DHCP_STOPPED:
      printf("DHCP in state %d\r", dhcp_status);
      break;

    case DHCP_RUNNING:
      printf("DHCP running\r");
      break;
    }

    flush_log();
    cli_poll();
  }
}

static void wizchip_dhcp_init(void) {
  printf(" DHCP client running\n");
  DHCP_init(SOCKET_DHCP, dhcpBuf);
  reg_dhcp_cbfunc(wizchip_dhcp_assign, wizchip_dhcp_assign, wizchip_dhcp_conflict);
}

static void wizchip_dhcp_assign(void) {
  getIPfromDHCP(g_net_info.ip);
  getGWfromDHCP(g_net_info.gw);
  getSNfromDHCP(g_net_info.sn);
  getDNSfromDHCP(g_net_info.dns);

  g_net_info.dhcp = NETINFO_DHCP;

  /* Network initialize */
  network_initialize(g_net_info); // apply from DHCP
  print_network_information(g_net_info);
  printf(" DHCP leased time : %d seconds\n", getDHCPLeasetime());
}

static void wizchip_dhcp_conflict(void) {
  printf(" Conflict IP from DHCP\n");

  // halt or reset or any...
  while (1)
    ; // this example is halt.
}

/* Timer */
static bool repeating_timer_callback(struct repeating_timer *t) {
  g_msec_cnt++;
  MilliTimer_Handler();

  if (g_msec_cnt >= 1000 - 1) {
    g_msec_cnt = 0;
    DHCP_time_handler();
    DNS_time_handler();
  }

  return true;
}

static bool cmpStr(char *bdata, size_t blength, char *cmpTo) { return strlen(cmpTo) == blength && memcmp(bdata, cmpTo, blength) == 0; }

static void processUpdateModbus(MessageData *msg) {
  printf("Processing Modbus update\n");
  char tmp_str[100];
  memcpy(tmp_str, msg->topicName->lenstring.data, msg->topicName->lenstring.len);
  tmp_str[msg->topicName->lenstring.len] = 0;
  char *ignored = strtok(tmp_str, "/");
  char *devid = strtok(NULL, "/");
  char *modbus = strtok(NULL, "/");
  char *devid_tok = strtok(NULL, "/");
  char *coil_tok = strtok(NULL, "/");

  if (strcmp(devid, switchDeviceID) != 0 || strcmp(modbus, "modbus") != 0 || !devid_tok || !coil_tok) {
    printf("Invlaid topic\n");
    return;
  }

  int device_id = atoi(devid_tok);
  int coil_id = atoi(coil_tok);
  int val = strncmp(msg->message->payload, "ON", 2) == 0 ? 1 : 0;
  modbus_set_coil(device_id, coil_id, val);
}

static void processUpdateDALI(MessageData *msg) {
  printf("Processing DALI update\n");
  char tmp_str[100];
  memcpy(tmp_str, msg->topicName->lenstring.data, msg->topicName->lenstring.len);
  tmp_str[msg->topicName->lenstring.len] = 0;
  char *ignored = strtok(tmp_str, "/");
  char *devid = strtok(NULL, "/");
  char *dalitok = strtok(NULL, "/");
  char *addrtok = strtok(NULL, "/");

  if (strcmp(devid, switchDeviceID) != 0 || strcmp(dalitok, "dali") != 0 || !addrtok || !addrtok) {
    printf("Invlaid topic\n");
    return;
  }
  int address = atoi(addrtok);

  cJSON *json = cJSON_ParseWithLength(msg->message->payload, msg->message->payloadlen);
  if (cJSON_IsObject(json)) {
    cJSON *stateVal = cJSON_GetObjectItemCaseSensitive(json, "state");

    if (cJSON_IsString(stateVal)) {
      printf("Dali device %d state set to %s\n", address, stateVal->valuestring);
      bool on = strcmp(stateVal->valuestring, "ON") == 0;
      dali_set_on(address, on);
    } else {
      printf("Invalid state value");
    }
  } else {
    printf("Invalid Message");
  }
  cJSON_Delete(json);
}

static void processUpdateBinding(MessageData *msg) {
  printf("Processing Status update message\n");
  char tmp_str[100];
  memcpy(tmp_str, msg->topicName->lenstring.data, msg->topicName->lenstring.len);
  tmp_str[msg->topicName->lenstring.len] = 0;

  char *ignored = strtok(tmp_str, "/");
  char *devid = strtok(NULL, "/");
  char *bindingtok = strtok(NULL, "/");
  char *devid_tok = strtok(NULL, "/");
  char *coil_tok = strtok(NULL, "/");

  if (strcmp(devid, switchDeviceID) != 0 || strcmp(bindingtok, "binding") != 0 || !devid_tok || !coil_tok) {
    printf("Invlaid topic\n");
    return;
  }

  int fixture_id = atoi(devid_tok);
  int button_id = atoi(coil_tok);

  binding_t binding;
  memcpy(tmp_str, msg->message->payload, msg->message->payloadlen);
  tmp_str[msg->message->payloadlen] = 0;
  if (!parse_binding(tmp_str, &binding)) {
    printf("Illegal binding value");
    return;
  }

  printf("Binding set for Fixutre %d, Button %d set to %d, %d, %d\n", fixture_id, button_id, binding.type, binding.device, binding.address);
  set_and_persist_binding(fixture_id, button_id, &binding);
}

static void processHAStatusMessage(MessageData *msg) {
  if (cmpStr(msg->message->payload, msg->message->payloadlen, "offline")) {
    printf("HA offline\n");
  } else if (cmpStr(msg->message->payload, msg->message->payloadlen, "online")) {
    printf("HA online\n");
    enumerate_all();

  } else {
    printf("Unknown HA status received: %.*s\n", msg->message->payloadlen, (char *)msg->message->payload);
  }
}
