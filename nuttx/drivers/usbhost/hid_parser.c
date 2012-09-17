/****************************************************************************
 * drivers/usbhost/hid_parser.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/usb/hid.h>
#include <nuttx/usb/hid_parser.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * Private Types
 ****************************************************************************/

struct hid_state_s
{
  struct hid_rptitem_attributes_s attrib;
  uint8_t rptcount;
  uint8_t id;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

int hid_parsereport(FAR const uint8_t *report, int rptlen,
                    hid_rptfilter_t filter, FAR struct hid_rptinfo_s *rptinfo)
{
  struct hid_state_s           state[CONFIG_HID_STATEDEPTH];
  struct hid_state_s          *currstate = &state[0];
  struct hid_collectionpath_s *collectionpath = NULL;
  struct hid_rptsizeinfo_s    *rptidinfo = &rptinfo->rptsize[0];
  uint16_t                     usage[CONFIG_HID_USAGEDEPTH];
  uint8_t                      nusage = 0;
  struct hid_range_s           usage_range = { 0, 0 };
  int                          i;

  DEBUGASSERT(report && filter && rptinfo);

  memset(rptinfo, 0x00, sizeof(struct hid_rptinfo_s));
  memset(currstate, 0x00, sizeof(struct hid_state_s));
  memset(rptidinfo, 0x00, sizeof(struct hid_rptsizeinfo_s));

  rptinfo->nreports = 1;

  while (rptlen > 0)
    {
      uint8_t  item = *report;
      uint32_t data = 0;

      report++;
      rptlen--;

      switch (item & USBHID_RPTITEM_SIZE_MASK)
        {
        case USBHID_RPTITEM_SIZE_4: /* 4 bytes of little endian data follow */
          data    = (uint32_t)(*report++);
          data   |= (uint32_t)(*report++) << 8;
          data   |= (uint32_t)(*report++) << 16;
          data   |= (uint32_t)(*report++) << 24;
          rptlen -= 4;
          break;

        case USBHID_RPTITEM_SIZE_2: /* 2 bytes of little endian data follow */
          data    = (uint32_t)(*report++);
          data   |= (uint32_t)(*report++) << 8;
          rptlen -= 2;
          break;

        case USBHID_RPTITEM_SIZE_1: /* 1 byte of data follows */
          data    = (uint32_t)(*report++);
          rptlen -= 1;
          break;

        case USBHID_RPTITEM_SIZE_0: /* No data follows */
        default:
          break;
        }

      switch (item & ~USBHID_RPTITEM_SIZE_MASK)
        {
        case USBHID_GLOBAL_PUSH_PREFIX:
          if (currstate == &state[CONFIG_HID_STATEDEPTH - 1])
            {
              return -E2BIG;
            }

          memcpy((currstate + 1),
                 currstate, sizeof(struct hid_rptitem_s));

          currstate++;
          break;

        case USBHID_GLOBAL_POP_PREFIX:
          if (currstate == &state[0])
            {
              return -EINVAL; /* Pop without push? */
            }

          currstate--;
          break;

        case USBHID_GLOBAL_USAGEPAGE_PREFIX:
          if ((item & USBHID_RPTITEM_SIZE_MASK) == USBHID_RPTITEM_SIZE_4)
            {
              currstate->attrib.usage.page = (data >> 16);
            }

          currstate->attrib.usage.page = data;
          break;

        case USBHID_GLOBAL_LOGICALMIN_PREFIX:
          currstate->attrib.logical.min = data;
          break;

        case USBHID_GLOBAL_LOGICALMAX_PREFIX:
          currstate->attrib.logical.max = data;
          break;

        case USBHID_GLOBAL_PHYSICALMIN_PREFIX:
          currstate->attrib.physical.min = data;
          break;

        case USBHID_GLOBAL_PHYSMICALAX_PREFIX:
          currstate->attrib.physical.max = data;
          break;

        case USBHID_GLOBAL_UNITEXP_PREFIX:
          currstate->attrib.unit.exponent = data;
          break;

        case USBHID_GLOBAL_UNIT_PREFIX:
          currstate->attrib.unit.type = data;
          break;

        case USBHID_GLOBAL_REPORTSIZE_PREFIX:
          currstate->attrib.bitsize = data;
          break;

        case USBHID_GLOBAL_REPORTCOUNT_PREFIX:
          currstate->rptcount = data;
          break;

        case USBHID_GLOBAL_REPORTID_PREFIX:
          currstate->id = data;

          if (rptinfo->haverptid)
            {
              rptidinfo = NULL;

              for (i = 0; i < rptinfo->nreports; i++)
                {
                  if (rptinfo->rptsize[i].id == currstate->id)
                    {
                      rptidinfo = &rptinfo->rptsize[i];
                      break;
                    }
                }

              if (rptidinfo == NULL)
                {
                  if (rptinfo->nreports == CONFIG_HID_MAXIDS)
                    {
                      return -EINVAL;
                    }

                  rptidinfo = &rptinfo->rptsize[rptinfo->nreports++];
                  memset(rptidinfo, 0x00, sizeof(struct hid_rptsizeinfo_s));
                }
            }

          rptinfo->haverptid = true;

          rptidinfo->id = currstate->id;
          break;

        case USBHID_LOCAL_USAGE_PREFIX:
          if (nusage == CONFIG_HID_USAGEDEPTH)
            {
              return -E2BIG;
            }

          usage[nusage++] = data;
          break;

        case USBHID_LOCAL_USAGEMIN_PREFIX:
          usage_range.min = data;
          break;

        case USBHID_LOCAL_USAGEMAX_PREFIX:
          usage_range.max = data;
          break;

        case USBHID_MAIN_COLLECTION_PREFIX:
          if (collectionpath == NULL)
            {
              collectionpath = &rptinfo->collectionpaths[0];
            }
          else
            {
              struct hid_collectionpath_s *ParentCollectionPath = collectionpath;

              collectionpath = &rptinfo->collectionpaths[1];

              while (collectionpath->parent != NULL)
                {
                  if (collectionpath == &rptinfo->collectionpaths[CONFIG_HID_MAXCOLLECTIONS - 1])
                    {
                      return -EINVAL;
                    }

                  collectionpath++;
                }

              collectionpath->parent = ParentCollectionPath;
            }

          collectionpath->type       = data;
          collectionpath->usage.page = currstate->attrib.usage.page;

          if (nusage)
            {
              collectionpath->usage.usage = usage[0];

              for (i = 0; i < nusage; i++)
                usage[i] = usage[i + 1];

              nusage--;
            }
          else if (usage_range.min <= usage_range.max)
            {
              collectionpath->usage.usage = usage_range.min++;
            }

          break;

        case USBHID_MAIN_ENDCOLLECTION_PREFIX:
          if (collectionpath == NULL)
            {
              return -EINVAL;
            }

          collectionpath = collectionpath->parent;
          break;
 
        case USBHID_MAIN_INPUT_PREFIX:
        case USBHID_MAIN_OUTPUT_PREFIX:
        case USBHID_MAIN_FEATURE_PREFIX:
          {
            int itemno;
            for (itemno = 0; itemno < currstate->rptcount; itemno++)
              {
                struct hid_rptitem_s newitem;
                uint8_t tag;

                memcpy(&newitem.attrib, &currstate->attrib,
                       sizeof(struct hid_rptitem_attributes_s));

                newitem.flags          = data;
                newitem.collectionpath = collectionpath;
                newitem.id             = currstate->id;

                if (nusage)
                  {
                    newitem.attrib.usage.usage = usage[0];

                    for (i = 0; i < nusage; i++)
                      {
                        usage[i] = usage[i + 1];
                      }
                    nusage--;
                  }
                else if (usage_range.min <= usage_range.max)
                  {
                    newitem.attrib.usage.usage = usage_range.min++;
                  }

                tag = (item & ~USBHID_RPTITEM_SIZE_MASK);
                if (tag == USBHID_MAIN_INPUT_PREFIX)
                  {
                    newitem.type = HID_REPORT_ITEM_IN;
                  }
                else if (tag == USBHID_MAIN_OUTPUT_PREFIX)
                  {
                    newitem.type = HID_REPORT_ITEM_OUT;
                  }
                else
                  {
                    newitem.type = HID_REPORT_ITEM_FEATURE;
                  }

                newitem.bitoffset              = rptidinfo->size[newitem.type];
                rptidinfo->size[newitem.type] += currstate->attrib.bitsize;

                /* Accumulate the maximum report size */

                if (rptinfo->maxrptsize < newitem.bitoffset)
                  {
                    rptinfo->maxrptsize = newitem.bitoffset;
                  }

                if ((data & USBHID_MAIN_CONSTANT) == 0 && filter(&newitem))
                  {
                    if (rptinfo->nitems == CONFIG_HID_MAXITEMS)
                      {
                        return -EINVAL;
                      }

                    memcpy(&rptinfo->items[rptinfo->nitems],
                           &newitem, sizeof(struct hid_rptitem_s));

                    rptinfo->nitems++;
                  }
              }
          }
          break;
        }

      if ((item & USBHID_RPTITEM_TYPE_MASK) == USBHID_RPTITEM_TYPE_MAIN)
        {
          usage_range.min = 0;
          usage_range.max = 0;
          nusage = 0;
        }
    }

  if (!(rptinfo->nitems))
    {
      return -ENOENT;
    }

  return OK;
}

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

int hid_getitem(FAR const uint8_t *report, FAR struct hid_rptitem_s *item)
{
  uint16_t remaining = item->attrib.bitsize;
  uint16_t offset    = item->bitoffset;
  uint32_t mask      = (1 << 0);

  if (item->id)
    {
      if (item->id != report[0])
        {
          return -ENOENT;
        }

      report++;
    }

  item->previous = item->value;
  item->value    = 0;

  while (remaining--)
    {
      if (report[offset >> 3] & (1 << (offset & 7)))
        {
          item->value |= mask;
        }

      offset++;
      mask <<= 1;
    }

  return OK;
}

/****************************************************************************
 * Name: hid_putitem
 *
 * Desription:
 *   Retrieves the given report item's value out of the Value member of the
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
void hid_putitem(FAR uint8_t *report, struct hid_rptitem_s *item)
{
  uint16_t remaining = item->attrib.bitsize;
  uint16_t offset    = item->bitoffset;
  uint32_t mask      = (1 << 0);

  if (item->id)
    {
      report[0] = item->id;
      report++;
    }

  item->previous = item->value;

  while (remaining--)
    {
      if (item->value & (1 << (offset & 7)))
        {
          report[offset >> 3] |= mask;
        }

      offset++;
      mask <<= 1;
    }
}
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
 
size_t hid_reportsize(FAR struct hid_rptinfo_s *rptinfo, uint8_t id, uint8_t rpttype)
{
  int i;
  for (i = 0; i < CONFIG_HID_MAXIDS; i++)
    {
      size_t size = rptinfo->rptsize[i].size[rpttype];

      if (rptinfo->rptsize[i].id == id)
        {
          return ((size >> 3) + ((size & 0x07) ? 1 : 0));
        }
    }

  return 0;
}
