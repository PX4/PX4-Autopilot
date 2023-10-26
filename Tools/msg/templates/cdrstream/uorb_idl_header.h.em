@{
import genmsg.msgs
import re

from px_generate_uorb_topic_helper import * # this is in Tools/

uorb_struct = '%s_s'%name_snake_case
uorb_struct_upper = name_snake_case.upper()
}@

/****************************************************************

  PX4 Cyclone DDS IDL to C Translator compatible idl struct
  Source:  @file_name_in
  Compatible with Cyclone DDS: V0.11.0

*****************************************************************/
#ifndef DDSC_IDL_UORB_@(uorb_struct_upper)_H
#define DDSC_IDL_UORB_@(uorb_struct_upper)_H

#include "dds/ddsc/dds_public_impl.h"
#include "dds/cdr/dds_cdrstream.h"
#include <uORB/topics/@(name_snake_case).h>

@##############################
@# Includes for dependencies
@##############################
@{
for field in spec.parsed_fields():
    if (not field.is_builtin):
        if (not field.is_header):
            (package, name) = genmsg.names.package_resource_name(field.base_type)
            package = package or spec.package # convert '' to package

            print('#include "%s.h"'%(name))
            name = re.sub(r'(?<!^)(?=[A-Z])', '_', name).lower()
            print('#include <uORB/topics/%s.h>'%(name))
}@


#ifdef __cplusplus
extern "C" {
#endif

@{
for field in spec.parsed_fields():
    if (not field.is_builtin):
        if (not field.is_header):
            (package, name) = genmsg.names.package_resource_name(field.base_type)
            package = package or spec.package # convert '' to package

            print('typedef px4_msg_%s px4_msg_px4__msg__%s;' % (name,name))
}@



typedef struct @uorb_struct px4_msg_@(file_base_name);

extern const struct dds_cdrstream_desc px4_msg_@(file_base_name)_cdrstream_desc;

#ifdef __cplusplus
}
#endif

#endif /* DDSC_IDL_UORB_@(uorb_struct_upper)_H */
