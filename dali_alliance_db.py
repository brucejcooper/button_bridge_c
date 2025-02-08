#!/usr/bin/env python
import os
from datetime import date
from typing import Iterable, NamedTuple
from xml.dom import Node, minidom
import re

import requests
import dateparser
from dom_query import select, select_all
import sqlite3



db = sqlite3.connect("products.db")
cur = db.cursor()
cur.execute("CREATE TABLE IF NOT EXISTS products(id PRIMARY KEY, brand, product, gtin, parts, init_reg, last_update)")
cur.close()


class DaliAllianceProductRecord(NamedTuple):
    brand_name: str
    product_name: str
    dali_parts: Iterable
    initial_registration: date
    last_updated: date


class DaliAllianceProductDB:
    """Fetches every product from the DALI alliance DB, and outputs it to a C file"""


    def node_text(self, e):
        """
        Why isn't there a default way to get the text of an element?  This is my poor man's version which simply concatenates any text in any subnode.
        """
        txt = ""

        if e.nodeType == Node.TEXT_NODE:
            txt = txt + e.data
        if e.hasChildNodes:
            for c in e.childNodes:
                txt = txt + " " + self.node_text(c)
        return txt.strip()

    def cast_to_datetime(self, v):
        if isinstance(v, str):
            return dateparser.parse(v)
        return v

    def to_dict(self, res) -> DaliAllianceProductRecord:
        return DaliAllianceProductRecord(
            brand_name = res[0],
            product_name =  res[1],
            dali_parts = [int(x) for x in res[2].split(", ")],
            initial_registration = self.cast_to_datetime(res[3]).now().date(),
            last_updated =  self.cast_to_datetime(res[4]).now().date(),
        )

    def update_gtins(self):

        res = db.execute("SELECT id from products where gtin is NULL")
        product_ids = [row[0] for row in res.fetchall()]
        res.close()

        for product_id in product_ids:
            print("Product", repr(product_id))

            details_res = requests.get(f'https://www.dali-alliance.org/products/{product_id}')
            doc = minidom.parseString(details_res.text)
            for r in  select_all(select(doc, 'table'), "tr"):
                (key, value) = [self.node_text(v) for v in select_all(r, "td")]
                if key == 'GTIN':
                    print("GTIN for product", product_id, "is", value)

                    db.execute('UPDATE products set gtin = ? where id = ?', (value, product_id))
                    db.commit()


    def dump(self):
        cur = db.execute("SELECT gtin,brand,product from products where gtin not null order by gtin")

        row = cur.fetchone()
        count = 0

        print('#include "dali_product_db.h"\n\nconst dali_product_db_t dali_product_db[] = {')

        tosort = []

        while row is not None:
            (gtin, brand, product) = row
            if gtin is not None and len(gtin) > 0:
                gtin = int(gtin)
                tosort.append((gtin, brand, product))
                count += 1
            row = cur.fetchone()

        tosort.sort(key=lambda x: x[0])

        for (gtin, brand, product) in tosort:
            print(f'{{ .gtin = {gtin}L, .brand = "{brand.replace('"', '\\"')}", .product = "{product.replace('"', '\\"')}"}},')


        print("};\nconst unsigned dali_product_db_sz = ", count, ";")


    def scan_dali_alliance(self):
        page = 1
        num_pages = 1


        while page <= num_pages:
            print("fetching page", page, "of", num_pages);
            response = requests.get(f'https://www.dali-alliance.org/products?module=ConsortiumProductPublic&command=Default&advanced_field=0&family_products=&page={page}')


            txt = response.text
            doc = minidom.parseString(txt)
            # Find the product table
            products = select_all(select(doc, 'table[class="product-listings"]'), "tbody > tr")

            pagetxt = self.node_text(select(doc, 'span[id="page-number"] > p'))
            m = re.match(r'Page (\d+) of (\d+)', pagetxt)
            page = int(m.group(1))
            num_pages = int(m.group(2))

            for product in products:
                product_attrs = {}
                product_id = product.getAttribute("data-id")
                for cell in select_all(product, "td"):
                    title = cell.getAttribute("data-title").lower()
                    if len(title) > 0:
                        product_attrs[title] = self.node_text(cell)

                        if title == "product name":
                            anchor = select(cell, "a")

                brand = product_attrs['brand name']
                prod_name = product_attrs['product name']
                parts = product_attrs['dali parts']
                init_reg = dateparser.parse(product_attrs['initial registration'])
                last_update = dateparser.parse(product_attrs['last updated'])

                cur = db.cursor()
                cur.execute(f'INSERT into products (id, brand, product, parts, init_reg, last_update) VALUES (?, ?, ?, ?, ?, ?) ON CONFLICT(id) DO NOTHING', (product_id, brand, prod_name, parts, init_reg, last_update))
                db.commit()
                cur.close()

                print(f"Brand {brand}, Name {prod_name}")
            page += 1

                            


DaliAllianceProductDB().dump()
