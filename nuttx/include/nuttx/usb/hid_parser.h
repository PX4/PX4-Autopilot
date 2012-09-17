/****************************************************************************
 * include/nuttx/usb/hid_parser.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *
 * Adapted from the LUFA Library:
 *
 *   Copyright 2011  Dean Camera (dean [at] fourwalledcubicle [dot] com)
 *   dean [at] fourwalledcubicle [dot] com, www.lufa-lib.org
 *
 * Permission to use, copy, modify, distribute, and sell this
 * software and its documentation for any purpose is hereby granted
 * without fee, provided that the above copyright notice appear in
 * all copies and that both that the copyright notice and this
 * permission notice and warranty disclaimer appear in supporting
 * documentation, and that the name of the author not be used in
 * advertising or publicity pertaining to distribution of the
 * software without specific, written prior permission.
 *
 * The author disclaim all warranties with regard to this
 * software, including all implied warranties of merchantability
 * and fitness.  In no event shall the author be liable for any
 * special, indirect or consequential damages or any damages
 * whatsoever resulting from loss of use, data or profits, whether
 * in an action of contract, negligence or other tortious action,
 * arising out of or in connection with the use or performance of
 * this software.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_USB_HID_PARSER_H
#define __INCLUDE_NUTTX_USB_HID_PARSER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_HID_STATEDEPTH
 *   Constant indicating the maximum stack depth of the state table. A larger
 *   state table allows for more PUSH/POP report items to be nested, but
 *   consumes more memory. By default this is set to 2 levels (allowing non-
 *   nested PUSH items) but this can be overridden by defining
 *   CONFIG_HID_STATEDEPTH in the Nuttx config file.
 *
 * CONFIG_HID_USAGEDEPTH
 *   Constant indicating the maximum stack depth of the usage table. A larger
 *   usage table allows for more USAGE items to be indicated sequentially for
 *   REPORT COUNT entries of more than one, but requires more stack space. By
 *   default this is set to 8 levels (allowing for a report item with a count
 *   of 8) but this can be overridden by defining CONFIG_HID_USAGEDEPTH to
 *   in the Nuttx config file.
 *
 * CONFIG_HID_MAXCOLLECTIONS
 *   Constant indicating the maximum number of COLLECTION items (nested or
 *   unnested) that can be processed in the report item descriptor. A large
 *   value allows for more COLLECTION items to be processed, but consumes
 *   more memory. By default this is set to 10 collections, but this can be
 *   overridden by defining CONFIG_HID_MAXCOLLECTIONS in the Nuttx config file.
 *
 * CONFIG_HID_MAXITEMS
 *   Constant indicating the maximum number of report items (IN, OUT or
 *   FEATURE) that can be processed in the report item descriptor and stored
 *   in the user HID Report Info structure. A large value allows
 *   for more report items to be stored, but consumes more memory. By default
 *   this is set to 20 items, but this can be overridden by defining
 *   CONFIG_HID_MAXITEMS in the Nuttx config file.
 *
 * CONFIG_HID_MAXIDS
 *   Constant indicating the maximum number of unique report IDs that can be
 *   processed in the report item descriptor for the report size information
 *   array in the user HID Report Info structure. A large value allows for
 *   more report ID report sizes to be stored, but consumes more memory. By
 *   default this is set to 10 items, but this can be overridden by defining
 *   CONFIG_HID_MAXIDS in the Nuttx config file. Note that IN, OUT and FEATURE
 *   items sharing the same report ID consume only one size item in the array.
 */

#ifndef CONFIG_HID_STATEDEPTH
#  define CONFIG_HID_STATEDEPTH      2
#endif

#ifndef CONFIG_HID_USAGEDEPTH
#  define CONFIG_HID_USAGEDEPTH      8
#endif

#ifndef CONFIG_HID_MAXCOLLECTIONS
#  define CONFIG_HID_MAXCOLLECTIONS 10
#endif

#ifndef CONFIG_HID_MAXITEMS
#  define CONFIG_HID_MAXITEMS       20
#endif

#ifndef CONFIG_HID_MAXIDS
#  define CONFIG_HID_MAXIDS         10
#endif

/* HID report type indices */

#define HID_REPORT_ITEM_IN           0
#define HID_REPORT_ITEM_OUT          1
#define HID_REPORT_ITEM_FEATURE      2
 
/****************************************************************************
 * Public Types
 ****************************************************************************/

/* HID Parser Report Item Min/Max Structure. Type define for an attribute
 * with both minimum and maximum values (e.g. Logical Min/Max).
 */

struct hid_range_s
{
  uint32_t min;                        /* Minimum value for the attribute */
  uint32_t max;                        /* Maximum value for the attribute */
};

/* HID Parser Report Item Unit Structure. Type define for the Unit attributes
 * of a report item.
 */

struct hid_unit_t
{
  uint32_t type;                       /* Unit type (refer to HID spec for details) */
  uint8_t  exponent;                   /* Unit exponent (refer to HID spec for details) */
};

/* HID Parser Report Item Usage Structure.  Type define for the Usage
 * attributes of a report item.
 */

struct hid_usage_t
{
  uint16_t page;                       /* Usage page of the report item */
  uint16_t usage;                      /* Usage of the report item */
};

/* HID Parser Report Item Collection Path Structure. Type define for a
 * COLLECTION object. Contains the collection attributes and a reference to
 * the parent collection if any.
 */

struct hid_collectionpath_s
{
  uint8_t type;                        /* Collection type (e.g. "Generic Desktop") */
  struct hid_usage_t usage;            /* Collection usage */
  struct hid_collectionpath_s *parent; /* Reference to parent collection (NULL if root) */
};

/* HID Parser Report Item Attributes Structure. Type define for all the data
 * attributes of a report item, except flags.
 */

struct hid_rptitem_attributes_s
{
  uint8_t bitsize;                    /* Size in bits of the report item's data */
  struct hid_usage_t usage;           /* Usage of the report item */
  struct hid_unit_t unit;             /* Unit type and exponent of the report item */
  struct hid_range_s logical;         /* Logical minimum and maximum of the report item */
  struct hid_range_s physical;        /* Physical minimum and maximum of the report item */
};

/* HID Parser Report Item Details Structure. Type define for a report item
 * (IN, OUT or FEATURE) layout attributes and other details.
 */

struct hid_rptitem_s
{
  uint16_t bitoffset;                 /* Bit offset in IN, OUT or FEATURE report of the item */
  uint8_t type;                       /* Report item type */
  uint16_t flags;                     /* Item data flags */
  uint8_t id;                         /* Report ID this item belongs to (0 if only one report) */
  struct hid_collectionpath_s *collectionpath;   /* Collection path of the item */
  struct hid_rptitem_attributes_s attrib; /* Report item attributes */
  uint32_t value;                     /* Current value of the report item */
  uint32_t previous;                  /* Previous value of the report item */
};

/* HID Parser Report Size Structure. Type define for a report item size
 * information structure, to retain the size of a device's reports by ID.
 */

struct hid_rptsizeinfo_s
{
  uint8_t id;                         /* Report ID of the report within the HID interface */
  uint16_t size[3];                   /* Number of bits in report type for the Report ID */
};

/* HID Parser State Structure. Type define for a complete processed HID
 * report, including all report item data and collections.
 */

struct hid_rptinfo_s
{
  /* nitems is the number of report items stored in the report items array (rptitems[]). */

  uint8_t nitems;
  struct hid_rptitem_s items[CONFIG_HID_MAXITEMS];

  /* All collection items, referenced by the report items. */

  struct hid_collectionpath_s collectionpaths[CONFIG_HID_MAXCOLLECTIONS];
 
  uint8_t nreports;                   /* Number of reports within the HID interface */
  struct hid_rptsizeinfo_s rptsize[CONFIG_HID_MAXIDS]; /* Report sizes for each report in the interface */
  uint16_t maxrptsize;                /* Largest report that the attached device will generate, in bits */
  bool haverptid;                     /* Device has at least one REPORT ID in its HID report */
};

/* Callback routine for the HID Report Parser. This callback must be
 * implemented by the user code when the parser is used, to determine what
 * report IN, OUT and FEATURE item's information is stored into the user
 * struct hid_rptinfo_s structure. This can be used to filter only those
 * items the application will be using, so that no RAM is wasted storing
 * the attributes for report items which will never be referenced by the
 * application.
 *
 * Input Parameters:
 *   item  Pointer to the current report item for user checking.
 *
 * Returned value:
 *   Boolean true if the item should be stored into the struct hid_rptinfo_s
 *   structure, false if it should be ignored.
 */

typedef bool (*hid_rptfilter_t)(FAR struct hid_rptitem_s *item);

/****************************************************************************
 * Public Function Protoypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
# define EXTERN extern "C"
extern "C" {
#else
# define EXTERN extern
#endif

/****************************************************************************
 * Name: hid_parsereport
 *
 * Description:
 *   Function to process a given HID report returned from an attached device,
 *   and store it into a given struct hid_rptinfo_s structure.
 *
 * Input Parameters:
 *    report   Buffer containing the device's HID report table.
 *    rptlen   Size in bytes of the HID report table.
 *    filter   Callback function to decide if an item should be retained
 *    rptinfo  Pointer to a struct hid_rptinfo_s instance for the parser output.
 *
 * Returned Value:
 *  Zero on success, otherwise a negated errno value.
 ****************************************************************************/

EXTERN int hid_parsereport(FAR const uint8_t *report, int rptlen,
                           hid_rptfilter_t filter,
                           FAR struct hid_rptinfo_s *rptinfo);

/****************************************************************************
 * Name: hid_getitem
 *
 * Description:
 *   Extracts the given report item's value out of the given HID report and
 *   places it into the value member of the report item's struct hid_rptitem_s
 *   structure.
 *
 *   When called on a report with an item that exists in that report, this
 *   copies the report item's Value to it's previous element for easy
 *   checking to see if an item's value has changed before processing a
 *   report. If the given item does not exist in the report, the function
 *   does not modify the report item's data.
 *
 * Input Parameters
 *   report  Buffer containing an IN or FEATURE report from an attached
 *               device.
 *   item        Pointer to the report item of interest in a struct hid_rptinfo_s
 *               item array.
 *
 * Returned Value:
 *   Zero on success, otherwise a negated errno value.
 *
 ****************************************************************************/

EXTERN int hid_getitem(FAR const uint8_t *report, FAR struct hid_rptitem_s *item);

/****************************************************************************
 * Name: hid_putitem
 *
 * Desription:
 *   Retrieves the given report item's value out of the value member of the
 *   report item's struct hid_rptitem_s structure and places it into the correct
 *   position in the HID report buffer. The report buffer is assumed to have
 *   the appropriate bits cleared before calling this function (i.e., the
 *   buffer should be explicitly cleared before report values are added).
 *
 *   When called, this copies the report item's Value element to it's
 *   previous element for easy checking to see if an item's value has
 *   changed before sending a report.
 *
 *   If the device has multiple HID reports, the first byte in the report is
 *   set to the report ID of the given item.
 *
 * Input Parameters:
 *   report  Buffer holding the current OUT or FEATURE report data.
 *   item    Pointer to the report item of interest in a struct hid_rptinfo_s
 *           item array.
 *
 ****************************************************************************/

#if 0 /* Not needed by host */
EXTERN void hid_putitem(FAR uint8_t *report, FAR struct hid_rptitem_s *item);
#endif

/****************************************************************************
 * Name: hid_reportsize
 *
 * Description:
 *   Retrieves the size of a given HID report in bytes from it's Report ID.
 *
 * InputParameters:
 *  rptinfo Pointer to a struct hid_rptinfo_s instance containing the parser output.
 *  id      Report ID of the report whose size is to be retrieved.
 *  rpttype Type of the report whose size is to be determined, a valued from the
 *          HID_ReportItemTypes_t enum.
 *
 *  Size of the report in bytes, or 0 if the report does not exist.
 *
 ****************************************************************************/
 
EXTERN size_t hid_reportsize(FAR struct hid_rptinfo_s *rptinfo,
                             uint8_t id, uint8_t rpttype);

#undef EXTERN
#if defined(__cplusplus)
}
#endif


#endif                          /* __INCLUDE_NUTTX_USB_HID_PARSER_H */
