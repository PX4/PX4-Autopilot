#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UTILS_STRING_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UTILS_STRING_OPERATIONS_GENERIC_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::utils::string{
    template <typename TI>
    TI length(const char* str, TI MAX_LEN) {
        TI len = 0;
        while (str[len] != '\0' && len < MAX_LEN) {
            len++;
        }
        return len;
    }
    template <typename TI>
    TI copy(char* dest, const char* src, TI n) {
        if (n == 0) {
            return 0;
        }
        for (TI i = 0; i < n; i++){
            dest[i] = src[i];
            if (src[i] == '\0'){
                return i;
            }
        }
        return n;
    }
    template <typename TI>
    RL_TOOLS_FUNCTION_PLACEMENT bool compare(const char* a, const char* b, TI n){
        TI i = 0;
        while (a[i] != '\0' && b[i] != '\0' && i < n) {
            if (a[i] != b[i]) return false;
            i++;
        }
        if (i < n){ // terminated because one of the strings stopped early. This implies that the other string also should stop early
            return a[i] == b[i];
        }
        return true;
    }
    template <typename T, typename TI>
    RL_TOOLS_FUNCTION_PLACEMENT void format_octal(char* dest, TI dest_size, T value) {
        // Convert value to octal and write to dest with zero-padding
        // dest_size includes the null terminator
        if (dest_size < 2) {
            if (dest_size == 1) {
                dest[0] = '\0';
            }
            return;
        }

        TI pos = dest_size - 2; // Start from the last position before null terminator
        dest[dest_size - 1] = '\0';

        if (value == 0) {
            for (TI i = 0; i < dest_size - 1; i++) {
                dest[i] = '0';
            }
            return;
        }

        // Fill with zeros first
        for (TI i = 0; i < dest_size - 1; i++) {
            dest[i] = '0';
        }

        // Convert to octal from right to left
        while (value > 0 && pos >= 0) {
            dest[pos] = '0' + (value & 7); // value % 8
            value >>= 3; // value / 8
            pos--;
        }
    }
    template <typename TI>
    RL_TOOLS_FUNCTION_PLACEMENT void memcpy(char* dest, const char* src, TI n) {
        for (TI i = 0; i < n; i++) {
            dest[i] = src[i];
        }
    }
    template <typename TI>
    TI parse_octal(const char* str, TI max_len) {
        // Parse octal string to integer
        // Skips leading spaces, stops at first non-octal digit or null terminator
        TI result = 0;
        TI i = 0;

        // Skip leading spaces
        while (i < max_len && str[i] == ' ') {
            i++;
        }

        // Parse octal digits (0-7)
        while (i < max_len && str[i] != '\0') {
            char c = str[i];
            if (c >= '0' && c <= '7') {
                result = (result << 3) | (c - '0'); // result * 8 + digit
            } else {
                // Stop at first non-octal character
                break;
            }
            i++;
        }

        return result;
    }
    template <typename T, typename TI>
    TI int_to_string(char* dest, TI dest_size, T value) {
        // Convert integer to decimal string
        // Returns the number of characters written (excluding null terminator)
        if (dest_size == 0) {
            return 0;
        }

        TI pos = 0;
        bool is_negative = false;

        // Handle zero specially
        if (value == 0) {
            if (dest_size > 1) {
                dest[0] = '0';
                dest[1] = '\0';
                return 1;
            } else {
                dest[0] = '\0';
                return 0;
            }
        }

        // Handle negative numbers
        if (value < 0) {
            is_negative = true;
            // Handle most negative value specially to avoid overflow
            // For two's complement: -MIN = MIN (overflow), so we handle it by processing digits directly
        }

        // Convert digits in reverse order
        char temp[32]; // Enough for any 64-bit integer
        TI temp_pos = 0;

        // Process digits (works for both positive and negative in two's complement)
        T working_value = is_negative ? value : value;
        while (working_value != 0 && temp_pos < 32) {
            // For negative numbers, modulo gives negative result, so take absolute of digit
            T digit = working_value % 10;
            if (digit < 0) digit = -digit;
            temp[temp_pos++] = '0' + digit;
            working_value /= 10;
        }

        // Add negative sign if needed
        if (is_negative && pos < dest_size - 1) {
            dest[pos++] = '-';
        }

        // Copy digits in correct order
        for (TI i = temp_pos; i > 0 && pos < dest_size - 1; i--) {
            dest[pos++] = temp[i - 1];
        }

        dest[pos] = '\0';
        return pos;
    }
    template <typename TI>
    TI string_to_int(const char* str, TI max_len) {
        // Parse decimal string to integer
        // Skips leading spaces, handles negative numbers
        TI result = 0;
        TI i = 0;
        bool is_negative = false;

        // Skip leading spaces
        while (i < max_len && str[i] == ' ') {
            i++;
        }

        // Check for negative sign
        if (i < max_len && str[i] == '-') {
            is_negative = true;
            i++;
        } else if (i < max_len && str[i] == '+') {
            i++;
        }

        // Parse decimal digits (0-9)
        while (i < max_len && str[i] != '\0') {
            char c = str[i];
            if (c >= '0' && c <= '9') {
                result = result * 10 + (c - '0');
            } else {
                // Stop at first non-decimal character
                break;
            }
            i++;
        }

        return is_negative ? -result : result;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
