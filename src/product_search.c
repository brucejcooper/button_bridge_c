

#include "dali_product_db.h"
#include <stddef.h>
#include "log.h"

extern const dali_product_db_t dali_product_db[];
extern const unsigned dali_product_db_sz;

/**
 * Does a binary search on the product db to get brand/product info out.  
 *  @returns NULL if there is no match, otherwise it will return a pointer to the DB record
 */
const dali_product_db_t *find_product_by_gtin(const uint64_t gtin) {
    int bottom = 0;
    int top = dali_product_db_sz-1;
    // defer_log("SEARCH", "testing %d %d for gtin %lld", bottom, top, gtin);
    while (bottom <= top) {
        int mid = (bottom+top)/2;
        const dali_product_db_t *midp =  dali_product_db + mid;
        // defer_log("SEARCH","trying index %d of %d = %llu", mid, top, midp->gtin);
        if (midp->gtin == gtin) {
            // defer_log("SEARCH", "GTIN match at %d", mid);
            return midp;
        }
        if (midp->gtin < gtin) {
            bottom = mid+1;
        } else {
            top = mid-1;
        }
    }
    defer_log("SEARCH", "No match, returning NULL");
    return NULL;

}