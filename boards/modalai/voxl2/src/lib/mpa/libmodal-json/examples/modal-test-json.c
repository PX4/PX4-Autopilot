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

#include <stdio.h>
#include <string.h>

#include <modal_json.h>


#define PATH	"/tmp/config/test1.json"

// variable to hold basic boolean
#define DEFAULT_BOOL1	0
static int bool1;

// variable to hold basic integer
#define DEFAULT_INT1	42
static int	int1;

// variable to hold basic double
#define DEFAULT_DOUBLE1	3.14159
static double	double1;

// variable to hold basic float
#define DEFAULT_FLOAT1	3.14159
static float	float1;

// reasonable maximum string length to hold simple strings that may be present
// in a config file such as modes and names. Pick one less than a round number
// so alocated arrays will be large enough to hold null terminator.
#define MAX_STRING_LEN	127
#define DEFAULT_STRING1	"ModalAI"
static char	string1[MAX_STRING_LEN+1]; // allow an extra byte for NULL terminator

// Can map to your own enumerated type if you want, but you still need to define
// the strings to search for in the json file. We use an int here for simplicity
#define DEFAULT_ENUM1	0
#define ENUM1_N_OPTIONS	4
static const char* enum1_options[] = {"option0","option1","option2","option3"};
static int enum1;

// fixed-length vector of doubles
#define FIXED_VECTOR1_LEN	3
static double default_fixed_vector1[] = {1.0, 2.0, 3.0};
static double fixed_vector1[FIXED_VECTOR1_LEN];

// fixed-length vector of floats
#define FIXED_VECTOR2_LEN	4
static float default_fixed_vector2[] = {4.0f, 5.0f, 6.0f, 7.0f};
static float fixed_vector2[FIXED_VECTOR2_LEN];

// dynamic vector will be populated up to the specified maximum length
// need to allocate enough memory to allow for up to max length.
// dynamic_vector1
#define DYNAMIC_VECTOR1_MAX_LEN	8
#define DEFAULT_DYNAMIC_VECTOR1_LEN	3
static double default_dynamic_vector1[] = {1.0, 2.0, 3.0};
static double dynamic_vector1[FIXED_VECTOR1_LEN];
static int dynamic_vector1_len;

// Matrix method assumes all data is in continuous memory, row-major order.
// If using an rc_matrix_t matrix then you can allocate it normally and point to
// the beginning of the first row.
#define MATRIX1_ROWS	4
#define MATRIX1_COLS	3
static double default_matrix1[MATRIX1_ROWS][MATRIX1_COLS] = {{ 11,  12,  13},
                                                             { 21,  22,  23},
                                                             { 31,  32,  33},
                                                             { 41,  42,  43}};
static double matrix1[MATRIX1_ROWS][MATRIX1_COLS];

// variables to hold 3 items in child_object_1
#define DEFAULT_BOOL2	1
static int bool2;
#define DEFAULT_INT2	3
static int	int2;
#define DEFAULT_DOUBLE2	2.71828
static double	double2;

// array1 is an array of strings
#define ARRAY1_MAX_LEN	8
static int array1_len;
static char array1[ARRAY1_MAX_LEN][MAX_STRING_LEN+1];

// array of structs to store apriltag information
#define MAX_APRILTAGS 64
typedef struct apriltag_t{
	int tag_id;						// ID of the tag
	char name[MAX_STRING_LEN+1];	// apriltag name, e.g. "landing pad"
	double T_tag_wrt_fixed[3];		// translation vector
	double R_tag_to_fixed[3][3];	// rotation matrix
} apriltag_t;
int n_apriltags;
apriltag_t apriltags[MAX_APRILTAGS];
// default position and orientation
double default_apriltag_T[3]	=	 { 0,  0,  0};
double default_apriltag_R[3][3]	=	{{ 0, -1,  0}, \
									 { 1,  0,  0}, \
									 { 0,  0,  1}};


/**
 * @brief      print out all the variables as-parsed to confirm the json file
 *             was loaded correctly.
 */
static void print_vars(void)
{
	int i,j,k;
	printf("bool1:            %d\n", bool1);
	printf("int1:             %d\n", int1);
	printf("double1:          %f\n", double1);
	printf("float1:           %f\n", (double)float1);
	printf("string1:          %s\n", string1);
	printf("enum1:            %s\n", enum1_options[enum1]);
	printf("fixed_vector1:   %4.1f %4.1f %4.1f\n", fixed_vector1[0], fixed_vector1[1], fixed_vector1[2]);
	printf("fixed_vector2:   %4.1f %4.1f %4.1f %4.1f\n", (double)fixed_vector2[0], (double)fixed_vector2[1], (double)fixed_vector2[2], (double)fixed_vector2[3]);
	printf("dynamic_vector1:");
	for(i=0;i<dynamic_vector1_len;i++) printf(" %4.1f",dynamic_vector1[i]);
	printf("\n");
	printf("matrix1:");
	for(i=0;i<MATRIX1_ROWS;i++){
		printf("\n                 ");
		for(j=0;j<MATRIX1_COLS;j++) printf(" %4.1f", matrix1[i][j]);
	}
	printf("\n");
	printf("child_object_1:\n");
	printf("    bool2:        %d\n", bool2);
	printf("    int2:         %d\n", int2);
	printf("    double2:      %f\n", double2);

	printf("\narray1 of %d strings:\n", array1_len);
	for(i=0;i<array1_len;i++) printf("                  %s\n", array1[i]);

	printf("\nn_apriltags:      %d\n", n_apriltags);
	for(i=0; i<n_apriltags; i++){
		printf("    apriltag #%d:\n",i);
		printf("        tag id:   %d\n", apriltags[i].tag_id);
		printf("        name:     %s\n", apriltags[i].name);
		printf("        T_tag_wrt_fixed:\n");
		printf("               ");
		for(j=0;j<3;j++) printf(" %4.1f",apriltags[i].T_tag_wrt_fixed[j]);
		printf("\n        R_tag_to_fixed:");
		for(j=0;j<3;j++){
			printf("\n               ");
			for(k=0;k<3;k++) printf(" %4.1f", apriltags[i].R_tag_to_fixed[j][k]);
		}
		printf("\n\n");
	}

	printf("\n");
	return;
}


/**
 * @brief      straightforward test of libmodal_json functions.
 *
 *             The program flow is as follows:
 *
 *             1. Make a new empty json file if one does not exist already. This
 *                will also create any missing directories in the specified file
 *                path.
 *
 *             2. Read the contents of the file into a top-level parent cJSON
 *                object and print the text to the screen.
 *
 *             3. Fetch some primitive types from the parent object. In each
 *                case, add the default value to the parent if the item is
 *                missing.
 *                 - boolean
 *                 - dnteger
 *                 - double
 *                 - string
 *                 - enum
 *                 - fixed-length vector
 *                 - dynamic-length vector
 *                 - fixed-size matrix
 *
 *             4. Fetch a child object. Using the cJSON* handle to this child
 *                object just like the parent object, use the same helper
 *                functions from before to fetch a few primitive values from
 *                this child object.
 *
 *             5. Fetch an array by name from the parent object. If its missing,
 *                add an empty array to the parent and populate with two default
 *                strings. Loop through each array item and copy out the string
 *                values.
 *
 *             6. So far we have tested primitives, objects, and arrays. Now try
 *                parsing through an array of objects containing four primitives
 *                each. For this we use apriltags as an example. Each apriltag
 *                is defined by a struct corresponding to its settings. If the
 *                apriltag array is missing from the file or just empty, add two
 *                default apritlag structs in.
 *
 *             7. Test removing a deprecated value from the parent object. This
 *                is useful if an old parameter is no longer needed or was
 *                updated with a new name.
 *
 *             8. Check if any parsing error since the beginning of this process
 *                and return an error if so. All of the "json_fetch_xxx" helpers
 *                set this error flag so you don't have to check the return
 *                value of every single function call which gets messy for big
 *                files.
 *
 *             9. Print out the contents of the local varibles that have been
 *                populated through this process to check the values are
 *                correct.
 *
 *             10. Check if the cJSON parent object was modified during this
 *                 process and re-write it back to disk if so.
 *
 *             11. Free the memory allocated for the cJSON objects, then return.
 *
 * @return     0 on success, -1 on failure
 */
int main(void)
{
	int i;

/**
 * Step 1
 *
 * Make a new empty json file if one does not exist already. This will also
 * create any missing directories in the specified file path.
 *
 * Allow for this program to start up wth a missing config file. Make a new one
 * if this is the case. We will populate it with defaults in the process of
 * parsing.
 */
	int ret = json_make_empty_file_if_missing(PATH);
	if(ret < 0) return -1;
	else if(ret>0) fprintf(stderr, "Created new empty json file: %s\n", PATH);


/**
 * Step 2
 *
 * Read the contents of the file into a top-level parent cJSON object and print
 * the text to the screen.
 *
 * This is the top level jSON object so we will call it "parent". Every other
 * item in the json file is also a cJSON object but we mostly don't have to
 * interact with those when using the helper functions. For more advanced
 * structures such as arrays then you will work with cJSON objects other than
 * "parent".
 *
 * Print the data to the screen so we can look at the raw file for inspection.
 */
	printf("\nOpening %s for reading\n\n", PATH);
	cJSON* parent = json_read_file(PATH);
	if(parent==NULL) return -1;
	printf("Read the following data from %s\n\n", PATH);
	json_print(parent);


/**
 * Step 3
 *
 * Fetch some primitive types from the parent object. In each case, add the
 * default value to the parent if the item is missing.
 *     - boolean
 *     - dnteger
 *     - double
 *     - string
 *     - enum
 *     - fixed-length vector
 *     - dynamic-length vector
 *     - fixed-size matrix
 *
 * If you don't want to add the default value, just use the equivalent function
 * without the "with_default" suffix.
 *
 * e.g.: json_fetch_bool() instead of json_fetch_bool_with_default()
 */
	printf("\n\nNow beginning to parse the data to local variables...\n");
	json_fetch_bool_with_default(	parent, "bool1",	&bool1,		DEFAULT_BOOL1);
	json_fetch_int_with_default(	parent, "int1",		&int1,		DEFAULT_INT1);
	json_fetch_double_with_default(	parent, "double1",	&double1,	DEFAULT_DOUBLE1);
	json_fetch_float_with_default(	parent, "float1",	&float1,	DEFAULT_FLOAT1);
	json_fetch_string_with_default(	parent, "string1",	string1,	MAX_STRING_LEN,	DEFAULT_STRING1);
	json_fetch_enum_with_default(	parent, "enum1",	&enum1,		enum1_options,	ENUM1_N_OPTIONS,	DEFAULT_ENUM1);
	json_fetch_fixed_vector_with_default(parent, "fixed_vector1",	fixed_vector1,	FIXED_VECTOR1_LEN,	default_fixed_vector1);
	json_fetch_fixed_vector_float_with_default(parent, "fixed_vector2",	fixed_vector2,	FIXED_VECTOR2_LEN,	default_fixed_vector2);
	json_fetch_dynamic_vector_with_default(parent, "dynamic_vector1", dynamic_vector1, &dynamic_vector1_len, DYNAMIC_VECTOR1_MAX_LEN, default_dynamic_vector1, DEFAULT_DYNAMIC_VECTOR1_LEN);
	json_fetch_fixed_matrix_with_default(parent, "matrix1", &matrix1[0][0], MATRIX1_ROWS, MATRIX1_COLS, &default_matrix1[0][0]);



/**
 * Step 4
 *
 * Fetch a child object. Using the cJSON* handle to this child object just like
 * the parent object, use the same helper functions from before to fetch a few
 * primitive values from this child object.
 *
 *
 * A child object can contain anything the parent object can. Here we make a
 * child object with just a bool, int, and double to demonstrate.
 */
	cJSON* child_object1 = json_fetch_object_and_add_if_missing(parent, "child_object1");
	json_fetch_bool_with_default(	child_object1, "bool2",		&bool2,		DEFAULT_BOOL2);
	json_fetch_int_with_default(	child_object1, "int2",		&int2,		DEFAULT_INT2);
	json_fetch_double_with_default(	child_object1, "double2",	&double2,	DEFAULT_DOUBLE2);


/**
 * Step 5
 *
 * Fetch an array by name from the parent object. If its missing, add an empty
 * array to the parent and populate with two default strings. Loop through each
 * array item and copy out the string values.
 *
 * Parsing arrays is not that difficult. The root of the array is a cJSON type
 * just like the parent and child objects. You can fetch its handle by name
 * using the json_fetch_array() or json_fetch_array_and_add_if_missing()
 * helpers. These helper functions also report the length of the array. Use the
 * cJSON_GetArrayItem() function to grab a handle to each item in the array.
 * Then you can use all the same helper functions that you used on the parent
 * object.
 *
 * Note that when we make the call to json_fetch_string() we pass in a handle to
 * the string item itself straight from the array and "NULL" for the name. This
 * is because items in an array do not have names like items in an object. The
 * json_fetch_xxx() helper functions support both methods: searching for an item
 * in an object by name, and fetching data from the item itself without
 * searching for it in another object by name.
 *
 * Here we arbitrarily set a limit to the array length of ARRAY1_MAX_LEN for
 * this fake config file. Arrays do not need a length limit.
 *
 * json_fetch_array_and_add_if_missing() will add an empty array (len=0) to the
 * parent object if its missing. The next step checks if the length==0. This
 * could be because the file originally contained an empty array, or because it
 * was just added by the json_fetch helper function. Either way, add two default
 * values to the array. Make sure to set the modified flag since we modified the
 * parent manually. The json_fetch_xxx helper functions will do this
 * automatically.
 */
	cJSON* array1_json = json_fetch_array_and_add_if_missing(parent, "array1", &array1_len);
	if(array1_len > ARRAY1_MAX_LEN){ // arbitrary length
		fprintf(stderr, "array1 should be an array of up to %d strings\n", ARRAY1_MAX_LEN);
		return -1;
	}
	// if array is empty, add some defaults
	if(array1_len == 0){
		cJSON_AddItemToArray(array1_json, cJSON_CreateString("array1_item1"));
		cJSON_AddItemToArray(array1_json, cJSON_CreateString("array1_item2"));
		array1_len = 2;
		json_set_modified_flag(1); // log that we modified the parent manually
	}
	// copy out each item in the array, note that array items have no name, so
	// we pass in NULL for the name argument to the json_fetch_xxx helper
	for(i=0; i<array1_len; i++){
		cJSON* item = cJSON_GetArrayItem(array1_json, i);
		json_fetch_string(item, NULL, array1[i], MAX_STRING_LEN);
	}



/**
 * Step 6.
 *
 * So far we have tested primitives, objects, and arrays. Now try parsing
 * through an array of objects, each containing four primitives. For this we use
 * apriltags as an example. Each apriltag is defined by a struct containing its
 * id, name, translation vector, and rotation matrix. If the apriltag array is
 * missing from the file or just empty, add two defaults in. This process is
 * just like going through the array of strings from before, except this time
 * each array item is an object with multiple named items instead of just a
 * string.
 *
 * This is the same structure as used by voxl-vision-px4 to describe fixed
 * apriltags.
 */

	cJSON* apriltag_json = json_fetch_array_and_add_if_missing(parent, "apriltags", &n_apriltags);
	if(n_apriltags > MAX_APRILTAGS){ // arbitrary length
		fprintf(stderr, "array of apriltags should be no more than %d long\n", MAX_APRILTAGS);
		return -1;
	}
	if(n_apriltags == 0){ // add two empty objects to the array. Defaults will be added by json_fetch_xxx helpers next
		cJSON_AddItemToArray(apriltag_json, cJSON_CreateObject());
		cJSON_AddItemToArray(apriltag_json, cJSON_CreateObject());
		n_apriltags = 2;
		json_set_modified_flag(1); // log that we modified the parent manually
	}
	// copy out each item in the array
	for(i=0; i<n_apriltags; i++){
		cJSON* item = cJSON_GetArrayItem(apriltag_json, i);
		json_fetch_int_with_default(item, "tag_id", &apriltags[i].tag_id, i);
		json_fetch_string_with_default(item, "name", apriltags[i].name, MAX_STRING_LEN, "default_name");
		json_fetch_fixed_vector_with_default(item, "T_tag_wrt_fixed", apriltags[i].T_tag_wrt_fixed, 3, default_apriltag_T);
		json_fetch_fixed_matrix_with_default(item, "R_tag_to_fixed", &apriltags[i].R_tag_to_fixed[0][0], 3, 3, &default_apriltag_R[0][0]);
	}


/**
 * Step 7
 *
 * Test removing a deprecated value from the parent object. This is useful if an
 * old parameter is no longer needed or was updated with a new name.
 *
 * Try adding a random entry called "deprecated1" to the file and see it get
 * removed.
 */
	if(json_remove_if_present(parent, "deprecated1")>0){
		printf("removed deprecated field from file\n");
	}


/**
 * Step 8
 *
 * Check if any parsing error since the beginning of this process and return an
 * error if so. All of the "json_fetch_xxx" helpers set this error flag so you
 * don't have to check the return value of every single function call which gets
 * messy for big files.
 */
	if(json_get_parse_error_flag()){
		fprintf(stderr, "failed to parse data\n");
		cJSON_Delete(parent);
		return -1;
	}

/**
 * Step 9
 *
 * Print out the contents of the local varibles that have been populated through
 * this process to check the values are correct.
 */
	printf("Successfully parsed the JSON data. Populated local variables as follows:\n\n");
	print_vars();

/**
 * Step 10
 *
 * Check if the cJSON parent object was modified during this process and
 * re-write it back to disk if so.
 *
 * All of the json_fetch_xxx_with_default() helper functions will have set the
 * modified flag if they added a default value in, indicating the file should be
 * re-written to disk. If you modify the parent yourself without the use of
 * helper functions then you can manually set this flag with
 * json_set_modified_flag(1). The library does nothing with the information
 * other than make it available to the user for this sort of decision.
 */
	// write modified data to disk if neccessary
	if(json_get_modified_flag()){
		printf("The JSON data was modified during parsing, saving the changes to disk\n");
		json_write_to_file(PATH, parent);
	}
	else{
		printf("The JSON data was not modified during parsing, no need to write back to disk\n");
	}

	printf("\nTry editing the file or deleting it\n");
	printf("then re-run this program to see the effect\n");



/**
 * Step 11
 *
 * Free the memory allocated for the cJSON objects, then return. A single call
 * to cJSON_Delete on the parent is sufficient as it will recursively throug
 * through all child obejcts. Even though we allocated memory through this
 * process when we created new arrays and objects, as long as they were added
 * back to the parent, they will be freed here.
 */
	cJSON_Delete(parent);
	return 0;
}
