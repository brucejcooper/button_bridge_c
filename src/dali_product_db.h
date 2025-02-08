#ifndef _CLI_H
#define _CLI_H
#include <stdint.h>
#include <stdbool.h>


typedef struct {
  uint64_t gtin;
  char *brand;
  char *product;
} dali_product_db_t;


const dali_product_db_t *find_product_by_gtin(const uint64_t gtin);

#endif
