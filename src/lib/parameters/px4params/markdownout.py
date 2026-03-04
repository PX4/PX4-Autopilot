from xml.sax.saxutils import escape
import codecs
import html

class MarkdownTablesOutput():
    def __init__(self, groups):
        result = (
"""# Parameter Reference

::: info
This documentation was auto-generated from the source code for this PX4 version (using `make parameters_metadata`).
:::

::: tip
If a listed parameter is missing from the Firmware see: [Finding/Updating Parameters](../advanced_config/parameters.md#parameter-not-in-firmware).
:::

<!-- markdown generator: src/lib/parameters/px4params/markdownout.py -->

"""
                  )

        # Define the groups and parameters that are specific to certain boards, to allow for a note to be added to the documentation.
        BOARD_SPECIFIC_GROUPS = ['Actuator Outputs']
        PARAM_IS_OUTPUT_TIMER = ['PWM_MAIN_TIM0', 'PWM_MAIN_TIM1', 'PWM_MAIN_TIM2', 'PWM_AUX_TIM0', 'PWM_AUX_TIM1', 'PWM_AUX_TIM2', 'PWM_AUX_TIM3']

        for group in groups:
            result += f'## {group.GetName()}\n\n'

            group_is_board_specific = (group.GetName() in BOARD_SPECIFIC_GROUPS)

            for param in group.GetParams():
                name = param.GetName()

                # Apply note if either the group is board specific
                apply_note_board_specific_group = group_is_board_specific

                # Apply note if name is in list of timer group parameters
                apply_timer_param_note = (name in PARAM_IS_OUTPUT_TIMER)

                short_desc = param.GetFieldValue("short_desc") or ''

                # Add fullstop to short_desc if not present
                if short_desc:
                    if not short_desc.strip().endswith('.'):
                        short_desc += "."

                long_desc = param.GetFieldValue("long_desc") or ''

                #Strip out short text from start of long text, if it ends in fullstop
                if long_desc.startswith(short_desc):
                    long_desc = long_desc[len(short_desc):].lstrip()

                min_val = param.GetFieldValue("min") or ''
                max_val = param.GetFieldValue("max") or ''
                increment = param.GetFieldValue("increment") or ''
                def_val = param.GetDefault() or ''
                unit = param.GetFieldValue("unit") or ''
                type = param.GetType()
                is_boolean = param.GetBoolean()
                reboot_required = param.GetFieldValue("reboot_required") or ''
                #board = param.GetFieldValue("board") or '' ## Disabled as no board values are defined in any parameters!
                #decimal = param.GetFieldValue("decimal") or '' #Disabled as is intended for GCS not people
                #field_codes = param.GetFieldCodes() ## Disabled as not needed for display.
                #boolean = param.GetFieldValue("boolean") # or '' # Disabled - does not appear useful.

                # Format values for display.
                enum_codes=param.GetEnumCodes() or '' # Gets numerical values for parameter.
                enum_output=''
                # Format codes and their descriptions for display.
                if enum_codes:
                    enum_output+='**Values:**\n\n'
                    enum_codes=sorted(enum_codes,key=float)
                    for item in enum_codes:
                        enum_output+=f"- `{item}`: {param.GetEnumValue(item)}\n"
                    enum_output+='\n\n'

                bitmask_list=param.GetBitmaskList() #Gets bitmask values for parameter
                bitmask_output=''
                #Format bitmask values
                if bitmask_list:
                    bitmask_output+='**Bitmask:**\n\n'
                    for bit in bitmask_list:
                        bit_text = param.GetBitmaskBit(bit)
                        bitmask_output+=f"- `{bit}`: {bit_text}\n"
                    bitmask_output+='\n\n'

                if is_boolean and def_val=='1':
                    def_val='Enabled (1)'
                if is_boolean and def_val=='0':
                    def_val='Disabled (0)'

                result += f'### {name} (`{type}`)' + ' {#' + name + '}\n\n'
                if apply_note_board_specific_group:
                    result += f'<Badge type="warning" text="This parameter is only present on some boards." />\n\n'
                if apply_timer_param_note:
                    result += f'<Badge type="info" text="The exact output numbers where this timer applies are board-specific." />\n\n'
                if short_desc:
                    result += f'{short_desc}\n\n'
                if long_desc:
                    result += f'{long_desc}\n\n'
                if enum_codes:
                    result += enum_output
                if bitmask_list:
                    result += bitmask_output
                # Format the ranges as a table.
                result += f"Reboot | minValue | maxValue | increment | default | unit\n--- | --- | --- | --- | --- | ---\n{'&check;' if reboot_required else '&nbsp;' } | {min_val} | {max_val} | {increment} | {def_val} | {unit} | \n\n"

        self.output = result

    def Save(self, filename):
        with codecs.open(filename, 'w', 'utf-8') as f:
            f.write(self.output)
