/*******************************************************************************
 * Copyright 2025 ModalAI Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 ******************************************************************************/


/**
 * libmodal_json is an extension of Dave Gamble's excellent cJSON library with
 * the following helper functions to facilitate the most common JSON file
 * operation used at ModalAI for config and calibration files. All the cJSON
 * functions and types are included in this header and library to give the user
 * complete low-level control.
 *
 * If there isn't a helper function in this header for something you want to do,
 * look through cJSON.h and you will find what you need!
 */

#ifndef MODAL_JSON_H
#define MODAL_JSON_H

// include all the powerful cJSON functions and types too!!
#include <cJSON.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief      Makes a new empty json file if one does not already exist at the
 *             specified path.
 *
 *             Use this in cases where a program may run before a config file
 *             has been made and you wish the program to populate the config
 *             file with default values itself on the first run. The empty json
 *             file simple consists of open and closed curly braces {} which is
 *             the minimum viable json syntax.
 *
 *             Note this also calls sync() to make sure the changes actually
 *             get flushed to disk to help prevent FS corruption.
 *
 * @param[in]  path  The absolute file path
 *
 * @return     0 if file already existed, 1 if a new file was successfully
 *             created, -1 on failure.
 */
int json_make_empty_file_if_missing(const char* path);


/**
 * @brief      same as json_make_empty_file_if_missing() but can optionally
 *             write a header before the empty curly braces.
 *
 *             The header must be a valid comment for it to be parsed back.
 *             Valid comments are the same as C and can be single or multi-line
 *             C comments. The header string should end in '\n'
 *
 *             Note this also calls sync() to make sure the changes actually
 *             get flushed to disk to help prevent FS corruption.
 *
 * @param[in]  path    The absolute file path
 * @param[in]  header  The header as a valid comment string ending in '\n'
 *
 * @return     0 if file already existed, 1 if a new file was successfully
 *             created, -1 on failure.
 */
int json_make_empty_file_with_header_if_missing(const char* path, const char* header);


/**
 * @brief      reads a json file and puts the data into a cJSON type
 *
 *             This will allocate new memory for the returned cJSON object. The
 *             user is responsible for freeing this memory after they are done
 *             using it with cJSON_delete().
 *
 * @param[in]  path  The absolute file path
 *
 * @return     pointer to new cJSON data or NULL on failure.
 */
cJSON* json_read_file(const char* path);


/**
 * @brief      writes json data to specified file
 *
 *             this overwrites any data already in the file.
 *
 *             Note this also calls sync() to make sure the changes actually get
 *             flushed to disk to help prevent FS corruption.
 *
 * @param[in]  path  The absolute file path
 * @param      json  The json object to write
 *
 * @return     0 on success, -1 on failure
 */
int json_write_to_file(const char* path, cJSON* json);


/**
 * @brief      same as json_write_to_file() but can optionally write a header
 *             before the json data.
 *
 *             The header must be a valid comment for it to be parsed back.
 *             Valid comments are the same as C and can be single or multi-line
 *             C comments. The header string should end in '\n'
 *
 *             Note this also calls sync() to make sure the changes actually
 *             get flushed to disk to help prevent FS corruption.
 *
 * @param[in]  path    The absolute file path
 * @param      obj     The object
 * @param[in]  header  The header
 * @param      json  The json object to write
 *
 * @return     0 on success, -1 on failure
 */
int json_write_to_file_with_header(const char* path, cJSON* obj, const char* header);


/**
 * @brief      print json data to stdout in the same form as would be written to a file.
 *
 * @param      json  The json object to print
 *
 * @return     0 on success, -1 on failure
 */
int json_print(cJSON* json);


/**
 * @brief      Get the parse error flag.
 *
 *             If at any point one of the json_fetch_xxx helpers encountered an
 *             error such as an item missing or of the wrong type, it will set
 *             this flag to 1 in addition to returning an error.
 *
 *             Use json_get_parse_error_flag() to check at the end of a sequence
 *             of json_fetch_xxx calls to see if any of them had an error. This
 *             prevents the need for checking the return value of every single
 *             json_fetch_xxx() call.
 *
 *             You may also set this flag manually with
 *             json_set_parse_error_flag() if you wish.
 *
 *             This flag will be set back to 0 on a call to json_read_file().
 *
 * @return     0 if there was no error, 1 if there was.
 */
int json_get_parse_error_flag(void);
void json_set_parse_error_flag(int flag);


/**
 * @brief      Get the modified flag.
 *
 *             If at any point one of the json_fetch_xxx_with_default or
 *             json_fetch_xxx_and_add_if_missing() helpers modified the parent,
 *             they will set the modified flag to 1.
 *
 *             Use json_get_modified_flag() to check at the end of a sequence of
 *             json_fetch_xxx calls to see if any of them modified the parent,
 *             indicating the data should be written back to disk with the new
 *             items.
 *
 *             You may also set this flag manually with json_set_modified_flag()
 *             if you wish.
 *
 *             This flag will be set back to 0 on a call to json_read_file().
 *
 * @return     0 if there was no modification, 1 if there was.
 */
int json_get_modified_flag(void);
void json_set_modified_flag(int flag);



/**
 * Helper functions for fetching a value. These functions can be used two ways:
 *
 *     1. Fetch an item's value from an object by name. In this case cJSON* obj
 *        points to a cJSON object (parent or child) that contains the desired
 *        value by name.
 *
 *     2. Fetch an item's value directly from the item itself. In this case
 *        cJSON* obj should be a pointer to the item itself you want to extract
 *        the value from. 'name' must be set to NULL. This use case is helpful
 *        when parsing arrays since you will likely loop through the array
 *        fetching each item by index. Items in an array DO NOT have names
 *        unlike items in an object.
 *
 * Information on specific types:
 *
 *     - bools are written out to an integer, 0 for false, 1 for true.
 *
 *     - ints are written out as ints.
 *
 *     - doubles are written out as doubles.
 *
 *     - strings are written out using strcpy. Make sure you provide a char* to
 *       enough memory for (maxlen+1) bytes to allow for the null terminator.
 *
 *     - For enums you provide an array of strings (options) along with the
 *       number of strings in the array (n_options). json_fetch_enum will try to
 *       match the string value to one of the provided options and writes back
 *       the index of the matched option. If the string contained in the file
 *       does not match an option, val is set to -1 and the function returns -1.
 *
 *     - fixed vectors are arrays of doubles. The array must be of known length.
 *       If the array in the file is missing, not the right length, or the wrong
 *       typethen this function will return -1.
 *
 *     - fixed vector floats are the same as fixed vectors but writes the data
 *       out to an array of single-precision floats insead of doubles. The
 *       array must be of known length. If the array in the file is missing, not
 *       the right length, or the wrong typethen this function will return -1.
 *
 *     - dynamic vectors are also arrays of doubles but allow for any length
 *       from 0 to maxlen. Be sure to provide an array long enough for maxlen
 *       doubles to be written into. The actually number of doubles found in the
 *       array is written out to len if len!=NULL
 *
 *     - fixed matrices are arrays of arrays of doubles and are written out in
 *       row-major order to the beginning of contiguous memory. See the example
 *       program.
 *
 *
 * Each of these functions sets the parse error flag to 1 if a problem in
 * encountered such as a missing value, entry is of the wrong type, or a fixed
 * vector/matrix is the wrong size. They all return 0 on success or -1 on
 * failure.
 */
int json_fetch_bool(cJSON* obj, const char* name, int* val);
int json_fetch_int(cJSON* obj, const char* name, int* val);
int json_fetch_double(cJSON* obj, const char* name, double* val);
int json_fetch_float(cJSON* obj, const char* name, float* val);
int json_fetch_string(cJSON* obj, const char* name, char* val, unsigned int maxlen);
int json_fetch_enum(cJSON* obj, const char* name, int* val, const char** options, int n_options);
int json_fetch_fixed_vector(cJSON* obj, const char* name, double* val, int len);
int json_fetch_fixed_vector_float(cJSON* obj, const char* name, float* val, int len);
int json_fetch_dynamic_vector(cJSON* obj, const char* name, double* val, int* len, int maxlen);
int json_fetch_fixed_matrix(cJSON* obj, const char* name, double* val, int rows, int cols);


/**
 * All of the following json_fetch_xxx_with_default() functions mimic the
 * behavior of the json_fetch_xxx() equivalent except they will add a new item
 * to the provided cJSON object with the given name and default value. These are
 * useful for config files which may expand with new features over time.
 *
 * If the provided name is missing from obj and the new default value is added
 * successfully then the modified flag is set to 1 indicating the cJSON data has
 * been modified and should be written back to disk with the new data. The data
 * won't be written back to disk automatically, you will need to do that
 * yourself after checking the flag with json_get_modified_flag().
 *
 * Since the purpose of these functions is to retrieve and optionally add a
 * named item to an object, cJSON* obj must be an object and the name string
 * cannot be left NULL as is possible with the non "with_default" fetch
 * functions.
 *
 * each function returns 0 on success or -1 on failure.
 */
int json_fetch_bool_with_default(cJSON* obj, const char* name, int* val, int default_val);
int json_fetch_int_with_default(cJSON* obj, const char* name, int* val, int default_val);
int json_fetch_double_with_default(cJSON* obj, const char* name, double* val, double default_val);
int json_fetch_float_with_default(cJSON* obj, const char* name, float* val, float default_val);
int json_fetch_string_with_default(cJSON* obj, const char* name, char* val, unsigned int maxlen, const char* default_val);
int json_fetch_enum_with_default(cJSON* obj, const char* name, int* val, const char** options, int n_options, int default_val);
int json_fetch_fixed_vector_with_default(cJSON* obj, const char* name, double* val, int len, double* default_val);
int json_fetch_fixed_vector_float_with_default(cJSON* obj, const char* name, float* val, int len, float* default_val);
int json_fetch_dynamic_vector_with_default(cJSON* obj, const char* name, double* val, int* len, int maxlen, double* default_val, int default_len);
int json_fetch_fixed_matrix_with_default(cJSON* obj, const char* name, double* val, int rows, int cols, double* default_val);


/**
 * Similar to the above fetch functions for basic data types, these functions
 * will fetch child objects, arrays, and arrays of objects. They are all
 * essentially wrappers around the cJSON_GetObjectItem() function provided by
 * the cJSON library, but with additional functionality and error prints. If you
 * are trying to retrieve an optional object or array and don't want the error
 * prints, then you can call the silent cJSON_GetObjectItem() function instead.
 *
 * Child objects can contain named basic types just like the parent object.
 *
 * Arrays contain non-named objects in order. When fetching an array, these
 * helpers will also write out the length of the array to aid in parsing.
 *
 * Similar to the "_with_default()" functions, the "_and_add_if_missing()"
 * equivalents here will add an empty object, empty array, or an array
 * containing one empty object to the cJSON* obj object and return a handle to
 * the new object/array. These just make it easier to add new default values
 * since an empty object/array will already have been created to add to.
 *
 * If you are constructing a json file complicated enough to use child objects
 * and array, you most likely want to make use of the powerful cJSON functions
 * included in this header/library. Rememember, modal_json is just a collection
 * of helper functions bundled with cJSON giving you low-level access to the
 * cJSON structure if you require it. You are in no way restricted to using just
 * these helper functions.
 *
 * See the example program for how to use these.
 *
 * All these functions return a handle to the desired object/array or NULL on
 * failure.
 */
cJSON* json_fetch_object(cJSON* obj, const char* name);
cJSON* json_fetch_object_and_add_if_missing(cJSON* obj, const char* name);
cJSON* json_fetch_array(cJSON* obj, const char* name, int* length);
cJSON* json_fetch_array_and_add_if_missing(cJSON* obj, const char* name, int* length);
cJSON* json_fetch_array_of_objects(cJSON* obj, const char* name, int* length);
cJSON* json_fetch_array_of_objects_and_add_if_missing(cJSON* obj, const char* name, int* length);


/**
 * @brief      Remove an item from the cJSON object if it is present
 *
 *             This is useful for old/deprecated values to be removed from the
 *             file. Note that it does not recursively search through child
 *             objects, it only looks in the object provided.
 *
 *             If the value is removed then the modified flag is set such that
 *             json_get_modified_flag() will return 1 if the value was
 *             removed.
 *
 * @param      obj   The object
 * @param[in]  name  The name of the item to remove
 * @param      json  The cJSON object
 *
 * @return     0 if item was not present, 1 if it was present and was deleted.
 */
int json_remove_if_present(cJSON* obj, const char* name);


/**
 * @brief      parse a yaml file into cJSON format. This is NOT a complete
 * and robust yaml parser, it's just enough to parse the opencv lens calibration
 * files.
 *
 * @param[in]  filename  The filename
 *
 * @return     pointer to a new cJSON object containing the parsed data
 */
cJSON* json_from_yaml(const char* filename);



#ifdef __cplusplus
}
#endif

#endif // end #define MODAL_JSON_H
