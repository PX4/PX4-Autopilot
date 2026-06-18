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
#include <stdlib.h>		// for malloc free
#include <unistd.h>		// for access()
#include <sys/stat.h>	// for mkdir
#include <limits.h>		// for PATH_MAX
#include <errno.h>

#include <modal_json.h>

int parse_error_flag = 0;
int modified_flag = 0;


int json_make_empty_file_if_missing(const char* path)
{
	return json_make_empty_file_with_header_if_missing(path, NULL);
}

int json_make_empty_file_with_header_if_missing(const char* path, const char* header)
{
	// return if file exists
	if(access(path, F_OK) != -1){
		return 0;
	}

	// make a new file
	cJSON* new_json = cJSON_CreateObject();
	if(new_json == NULL){
		fprintf(stderr, "ERROR: in %s, failed to make new cJSON object\n", __FUNCTION__);
		return -1;
	}
	if(json_write_to_file_with_header(path, new_json, header)){
		fprintf(stderr, "ERROR: in %s, failed to write to disk\n", __FUNCTION__);
		return -1;
	}
	cJSON_Delete(new_json);
	return 1;
}


cJSON* json_read_file(const char* path)
{
	FILE* fd = NULL;
	char* data = NULL;
	int len;
	cJSON* new_json;

	const char *error_ptr;

	// check file exists
	if(access(path, F_OK) == -1){
		fprintf(stderr,"ERROR: in %s, file is missing: %s\n", __FUNCTION__, path);
		return NULL;
	}
	// open for reading
	fd = fopen(path, "r");
	if(fd==NULL){
		fprintf(stderr,"ERROR: in %s, can't open file %s\n", __FUNCTION__, path);
		fprintf(stderr,"Run voxl-configure-vision-px4\n");
		return NULL;
	}
	// check how long the file is
	fseek(fd,0,SEEK_END);
	len=ftell(fd);
	if(len<=0){
		fprintf(stderr,"ERROR: in %s, config file is empty or unreadable\n", __FUNCTION__);
		fclose(fd);
		return NULL;
	}
	// allocate memory to hold the file data
	data=malloc(len);
	if(data==NULL){
		fprintf(stderr,"ERROR: in %s, failed to allocate %d bytes for config file data\n", __FUNCTION__, len);
		fclose(fd);
		return NULL;
	}
	// read data in
	fseek(fd,0,SEEK_SET);
	size_t read_bytes = fread(data,1,len,fd);
	if ((int)read_bytes != len) {
		fprintf(stderr, "ERROR: expected %d bytes, read %zu\n", len, read_bytes);
	}

	fclose(fd);
	// minify to remove comments and stray characters
	cJSON_Minify(data);
	// parse into cJSON linked list
	new_json = cJSON_ParseWithLength(data,len);
	free(data);
	if(new_json == NULL){
		// TODO: improve this error message. This is an open issue in the cJSON
		// github page, hopefully a PR is submitted and accepted soon.
		fprintf(stderr, "Error in %s while parsing file %s\n", __FUNCTION__, path);
		fprintf(stderr, "The syntax error occured immediately BEFORE the following line:\n");
		error_ptr = cJSON_GetErrorPtr();
		if(error_ptr != NULL){
			// count characters up to the next line and print
			int i;
			for(i=0; error_ptr[i]; i++) {
				if(error_ptr[i]=='\n') break;
			}
			fprintf(stderr, "%.*s\n", i, error_ptr);
		}
		return NULL;
	}

	// fresh file has been opened, set flags to 0;
	parse_error_flag = 0;
	modified_flag = 0;
	return new_json;
}


static int _mkdir(const char *dir)
{
	char tmp[PATH_MAX];
	char* p = NULL;

	snprintf(tmp, sizeof(tmp),"%s",dir);
	for(p = tmp + 1; *p!=0; p++){
		if(*p == '/'){
			*p = 0;
			if(mkdir(tmp, S_IRWXU) && errno!=EEXIST){
				perror("ERROR calling mkdir");
				printf("tried to mkdir %s\n", tmp);
				return -1;
			}
			*p = '/';
		}
	}
	return 0;
}


int json_write_to_file(const char* path, cJSON* obj)
{
	return json_write_to_file_with_header(path, obj, NULL);
}


int json_write_to_file_with_header(const char* path, cJSON* obj, const char* header)
{
	// sanity checks
	if(obj==NULL){
		fprintf(stderr, "ERROR: in %s, received NULL pointer\n", __FUNCTION__);
	}

	// convert json object to a string, this allocates memory!
	char* new_string = cJSON_Print(obj);
	if(new_string == NULL){
		fprintf(stderr, "ERROR: in %s, failed to write JSON data to string\n", __FUNCTION__);
		return -1;
	}

	// make sure the path exists
	if(_mkdir(path)){
		fprintf(stderr, "ERROR: in %s, failed to create path %s\n", __FUNCTION__, path);
		return -1;
	}

	// open for writing
	FILE* fd = fopen(path, "w");
	if(fd==NULL){
		fprintf(stderr,"ERROR: in %s, can't open file %s for writing\n", __FUNCTION__, path);
		perror("");
		free(new_string);
		return -1;
	}

	// write header to disk if it exists
	if(header!=NULL && fputs(header,fd)<0){
		fprintf(stderr, "ERROR: in %s writing header to file\n", __FUNCTION__);
		perror("");
		free(new_string);
		fclose(fd);
		return -1;
	}

	// write to disk
	if(fputs(new_string,fd)<0){
		fprintf(stderr, "ERROR: in %s writing json string to file\n", __FUNCTION__);
		perror("");
		free(new_string);
		fclose(fd);
		return -1;
	}

	// put a terminating newline char
	fputs("\n",fd);

	// free memory and close
	free(new_string);
	fclose(fd);

	// sync the file system. This may seem extreme, but most of our json config
	// files live in the root file system and people have a habit of editing them
	// and immediately pulling the system power causing corrupted file systems.
	// So we do our best by syncing frequently.
	sync();

	return 0;
}


int json_print(cJSON* obj)
{
	// sanity checks
	if(obj==NULL){
		fprintf(stderr, "ERROR: in %s, received NULL pointer\n", __FUNCTION__);
	}

	// convert json object to a string, this allocates memory!
	char* new_string = cJSON_Print(obj);
	if(new_string == NULL){
		fprintf(stderr, "ERROR: in %s, failed to write JSON data to string\n", __FUNCTION__);
		return -1;
	}

	// print the string
	printf("%s", new_string);

	// free memory and return
	free(new_string);
	return 0;
}


int json_get_parse_error_flag(void)
{
	return parse_error_flag;
}


int json_get_modified_flag(void)
{
	return modified_flag;
}

void json_set_parse_error_flag(int flag)
{
	parse_error_flag = flag;
}

void json_set_modified_flag(int flag)
{
	modified_flag = flag;
}



int json_fetch_bool(cJSON* obj, const char* name, int* val)
{
	cJSON* item;
	// The user can provide the cjson pointer to the exact item to be fetched or
	// to an object that contains the item by name. Make the choice here:
	if(name==NULL) item = obj;	// look at the current cjson object itself
	else{						// otherwise look for the item by name
		item = cJSON_GetObjectItem(obj, name);
		if(item==NULL){
			fprintf(stderr, "ERROR: object missing %s\n", name);
			parse_error_flag = 1;
			return -1;
		}
	}
	// now read out data into user's variable
	if(cJSON_IsFalse(item)) *val=0;
	else if(cJSON_IsTrue(item)) *val=1;
	else{
		fprintf(stderr, "ERROR: parsing json object: %s, should be a boolean\n", name);
		parse_error_flag = 1;
		return -1;
	}
	return 0;
}


int json_fetch_bool_with_default(cJSON* obj, const char* name, int* val, int default_val)
{
	cJSON* item = cJSON_GetObjectItem(obj, name);
	if(item==NULL){
		item = cJSON_AddBoolToObject(obj, name, default_val);
		if(item==NULL){
			fprintf(stderr, "ERROR: could not add %s to JSON object\n", name);
			parse_error_flag = 1;
			return -1;
		}
		modified_flag = 1;
	}
	return json_fetch_bool(item, NULL, val);
}


int json_fetch_int(cJSON* obj, const char* name, int* val)
{
	cJSON* item;
	// The user can provide the cjson pointer to the exact item to be fetched or
	// to an object that contains the item by name. Make the choice here:
	if(name==NULL) item = obj;	// look at the current cjson object itself
	else{						// otherwise look for the item by name
		item = cJSON_GetObjectItem(obj, name);
		if(item==NULL){
			fprintf(stderr, "ERROR: object missing %s\n", name);
			parse_error_flag = 1;
			return -1;
		}
	}

	if(!cJSON_IsNumber(item)){
		fprintf(stderr, "ERROR: parsing json object: %s, should be an integer\n", name);
		parse_error_flag = 1;
		return -1;
	}
	*val = item->valueint;
	return 0;
}


int json_fetch_int_with_default(cJSON* obj, const char* name, int* val, int default_val)
{
	cJSON* item = cJSON_GetObjectItem(obj, name);
	if(item==NULL){
		item = cJSON_AddNumberToObject(obj, name, default_val);
		if(item==NULL){
			fprintf(stderr, "ERROR: could not add %s to JSON object\n", name);
			parse_error_flag = 1;
			return -1;
		}
		modified_flag = 1;
	}
	return json_fetch_int(item, NULL, val);
}


int json_fetch_double(cJSON* obj, const char* name, double* val)
{
	cJSON* item;
	// The user can provide the cjson pointer to the exact item to be fetched or
	// to an object that contains the item by name. Make the choice here:
	if(name==NULL) item = obj;	// look at the current cjson object itself
	else{						// otherwise look for the item by name
		item = cJSON_GetObjectItem(obj, name);
		if(item==NULL){
			fprintf(stderr, "ERROR: object missing %s\n", name);
			parse_error_flag = 1;
			return -1;
		}
	}

	if(!cJSON_IsNumber(item)){
		fprintf(stderr, "ERROR: parsing json object: %s, should be a double\n", name);
		parse_error_flag = 1;
		return -1;
	}
	*val = item->valuedouble;
	return 0;
}


int json_fetch_double_with_default(cJSON* obj, const char* name, double* val, double default_val)
{
	cJSON* item = cJSON_GetObjectItem(obj, name);
	if(item==NULL){
		item = cJSON_AddNumberToObject(obj, name, default_val);
		if(item==NULL){
			fprintf(stderr, "ERROR: could not add %s to JSON object\n", name);
			parse_error_flag = 1;
			return -1;
		}
		modified_flag = 1;
	}
	return json_fetch_double(item, NULL, val);
}


int json_fetch_float(cJSON* obj, const char* name, float* val)
{
	cJSON* item;
	// The user can provide the cjson pointer to the exact item to be fetched or
	// to an object that contains the item by name. Make the choice here:
	if(name==NULL) item = obj;	// look at the current cjson object itself
	else{						// otherwise look for the item by name
		item = cJSON_GetObjectItem(obj, name);
		if(item==NULL){
			fprintf(stderr, "ERROR: object missing %s\n", name);
			parse_error_flag = 1;
			return -1;
		}
	}

	if(!cJSON_IsNumber(item)){
		fprintf(stderr, "ERROR: parsing json object: %s, should be a double\n", name);
		parse_error_flag = 1;
		return -1;
	}
	*val = (float)item->valuedouble;
	return 0;
}


int json_fetch_float_with_default(cJSON* obj, const char* name, float* val, float default_val)
{
	cJSON* item = cJSON_GetObjectItem(obj, name);
	if(item==NULL){
		item = cJSON_AddNumberToObject(obj, name, default_val);
		if(item==NULL){
			fprintf(stderr, "ERROR: could not add %s to JSON object\n", name);
			parse_error_flag = 1;
			return -1;
		}
		modified_flag = 1;
	}
	return json_fetch_float(item, NULL, val);
}


int json_fetch_string(cJSON* obj, const char* name, char* val, unsigned int maxlen)
{
	cJSON* item;
	// The user can provide the cjson pointer to the exact item to be fetched or
	// to an object that contains the item by name. Make the choice here:
	if(name==NULL) item = obj;	// look at the current cjson object itself
	else{						// otherwise look for the item by name
		item = cJSON_GetObjectItem(obj, name);
		if(item==NULL){
			fprintf(stderr, "ERROR: object missing %s\n", name);
			parse_error_flag = 1;
			return -1;
		}
	}

	if(!cJSON_IsString(item)){
		fprintf(stderr, "ERROR: parsing json object: %s, should be a string\n", name);
		parse_error_flag = 1;
		return -1;
	}
	if(strlen(item->valuestring)>maxlen){
		fprintf(stderr, "ERROR: parsing json object: length of string %s should be <= %d\n", name, maxlen);
		parse_error_flag = 1;
		return -1;
	}
	strcpy(val,item->valuestring);
	return 0;
}


int json_fetch_string_with_default(cJSON* obj, const char* name, char* val, unsigned int maxlen, const char* default_val)
{
	cJSON* item = cJSON_GetObjectItem(obj, name);
	if(item==NULL){
		item = cJSON_AddStringToObject(obj, name, default_val);
		if(item==NULL){
			fprintf(stderr, "ERROR: could not add %s to JSON object\n", name);
			parse_error_flag = 1;
			return -1;
		}
		modified_flag = 1;
	}
	return json_fetch_string(item, NULL, val, maxlen);
}


int json_fetch_enum(cJSON* obj, const char* name, int* val, const char** options, int n_options)
{
	cJSON* item;
	// The user can provide the cjson pointer to the exact item to be fetched or
	// to an object that contains the item by name. Make the choice here:
	if(name==NULL) item = obj;	// look at the current cjson object itself
	else{						// otherwise look for the item by name
		item = cJSON_GetObjectItem(obj, name);
		if(item==NULL){
			fprintf(stderr, "ERROR: object missing %s\n", name);
			parse_error_flag = 1;
			return -1;
		}
	}

	if(!cJSON_IsString(item)){
		fprintf(stderr, "ERROR: parsing json object: %s, should be a string\n", name);
		parse_error_flag = 1;
		return -1;
	}
	// compare found string with each option until we find a match
	int found=0;
	int i;
	for(i=0; i<n_options; i++){
		if(strcmp(item->valuestring, options[i])==0){
			*val=i;
			found=1;
			break;
		}
	}
	if(!found){
		*val = -1;
		fprintf(stderr,"ERROR: parsing json object: invalid option for item %s\n", name);
		fprintf(stderr,"valid options are:\n");
		for(i=0; i<n_options; i++) fprintf(stderr, "%s\n", options[i]);
		return -1;
	}
	return 0;
}


int json_fetch_enum_with_default(cJSON* obj, const char* name, int* val, const char** options, int n_options, int default_val)
{
	cJSON* item = cJSON_GetObjectItem(obj, name);
	if(item==NULL){
		item = cJSON_AddStringToObject(obj, name, options[default_val]);
		if(item==NULL){
			fprintf(stderr, "ERROR: could not add %s to JSON object\n", name);
			parse_error_flag = 1;
			return -1;
		}
		modified_flag = 1;
	}
	return json_fetch_enum(item, NULL, val, options, n_options);
}


int json_fetch_fixed_vector(cJSON* obj, const char* name, double* val, int len)
{
	cJSON* item;
	// The user can provide the cjson pointer to the exact item to be fetched or
	// to an object that contains the item by name. Make the choice here:
	if(name==NULL) item = obj;	// look at the current cjson object itself
	else{						// otherwise look for the item by name
		item = cJSON_GetObjectItem(obj, name);
		if(item==NULL){
			fprintf(stderr, "ERROR: object missing %s\n", name);
			parse_error_flag = 1;
			return -1;
		}
	}

	if(!cJSON_IsArray(item)){
		fprintf(stderr, "ERROR: parsing json object: %s should be an array\n", name);
		parse_error_flag = 1;
		return -1;
	}
	else if(cJSON_GetArraySize(item)!=len){
		fprintf(stderr, "ERROR: parsing json object: %s expected array length %d\n", name, len);
		parse_error_flag = 1;
		return -1;
	}
	else{
		for(int i=0; i<len; i++){
			val[i]=cJSON_GetArrayItem(item, i)->valuedouble;
		}
	}
	return 0;
}


int json_fetch_fixed_vector_with_default(cJSON* obj, const char* name, double* val, int len, double* default_val)
{
	cJSON* item = cJSON_GetObjectItem(obj, name);
	if(item==NULL){
		item = cJSON_CreateDoubleArray(default_val, len);
		if(item==NULL){
			fprintf(stderr, "ERROR: could not add %s to JSON object\n", name);
			parse_error_flag = 1;
			return -1;
		}
		cJSON_AddItemToObject(obj, name, item);
		modified_flag = 1;
	}
	json_fetch_fixed_vector(item, NULL, val, len);
	return 0;
}


int json_fetch_fixed_vector_float(cJSON* obj, const char* name, float* val, int len)
{
	cJSON* item;
	// The user can provide the cjson pointer to the exact item to be fetched or
	// to an object that contains the item by name. Make the choice here:
	if(name==NULL) item = obj;	// look at the current cjson object itself
	else{						// otherwise look for the item by name
		item = cJSON_GetObjectItem(obj, name);
		if(item==NULL){
			fprintf(stderr, "ERROR: object missing %s\n", name);
			parse_error_flag = 1;
			return -1;
		}
	}

	if(!cJSON_IsArray(item)){
		fprintf(stderr, "ERROR: parsing json object: %s should be an array\n", name);
		parse_error_flag = 1;
		return -1;
	}
	else if(cJSON_GetArraySize(item)!=len){
		fprintf(stderr, "ERROR: parsing json object: %s expected array length %d\n", name, len);
		parse_error_flag = 1;
		return -1;
	}
	else{
		for(int i=0; i<len; i++){
			val[i] = (float)cJSON_GetArrayItem(item, i)->valuedouble;
		}
	}
	return 0;
}


int json_fetch_fixed_vector_float_with_default(cJSON* obj, const char* name, float* val, int len, float* default_val)
{
	cJSON* item = cJSON_GetObjectItem(obj, name);
	if(item==NULL){
		item = cJSON_CreateFloatArray(default_val, len);
		if(item==NULL){
			fprintf(stderr, "ERROR: could not add %s to JSON object\n", name);
			parse_error_flag = 1;
			return -1;
		}
		cJSON_AddItemToObject(obj, name, item);
		modified_flag = 1;
	}
	json_fetch_fixed_vector_float(item, NULL, val, len);
	return 0;
}


int json_fetch_dynamic_vector(cJSON* obj, const char* name, double* val, int* len, int maxlen)
{
	cJSON* item;
	// The user can provide the cjson pointer to the exact item to be fetched or
	// to an object that contains the item by name. Make the choice here:
	if(name==NULL) item = obj;	// look at the current cjson object itself
	else{						// otherwise look for the item by name
		item = cJSON_GetObjectItem(obj, name);
		if(item==NULL){
			fprintf(stderr, "ERROR: object missing %s\n", name);
			parse_error_flag = 1;
			return -1;
		}
	}

	if(!cJSON_IsArray(item)){
		fprintf(stderr, "ERROR: parsing json object: %s should be an array\n", name);
		parse_error_flag = 1;
		return -1;
	}
	int n = cJSON_GetArraySize(item);
	if(n>maxlen){
		fprintf(stderr, "ERROR: parsing json object: %s array exceeds maximum length of %d\n", name, maxlen);
		parse_error_flag = 1;
		return -1;
	}
	for(int i=0; i<n; i++){
		val[i]=cJSON_GetArrayItem(item, i)->valuedouble;
	}
	if(len!=NULL) *len = n;
	return 0;
}


int json_fetch_dynamic_vector_with_default(cJSON* obj, const char* name, double* val, int* len, int maxlen, double* default_val, int default_len)
{
	cJSON* item = cJSON_GetObjectItem(obj, name);
	if(item==NULL){
		item = cJSON_CreateDoubleArray(default_val, default_len);
		if(item==NULL){
			fprintf(stderr, "ERROR: could not add %s to JSON object\n", name);
			parse_error_flag = 1;
			return -1;
		}
		cJSON_AddItemToObject(obj, name, item);
		modified_flag = 1;
	}
	json_fetch_dynamic_vector(item, NULL, val, len, maxlen);
	return 0;
}


int json_fetch_fixed_matrix(cJSON* obj, const char* name, double* val, int rows, int cols)
{
	cJSON* item;
	// The user can provide the cjson pointer to the exact item to be fetched or
	// to an object that contains the item by name. Make the choice here:
	if(name==NULL) item = obj;	// look at the current cjson object itself
	else{						// otherwise look for the item by name
		item = cJSON_GetObjectItem(obj, name);
		if(item==NULL){
			fprintf(stderr, "ERROR: object missing %s\n", name);
			parse_error_flag = 1;
			return -1;
		}
	}

	if(!cJSON_IsArray(item)){
		fprintf(stderr, "ERROR: parsing json object: %s should be a matrix with %d rows and %d columns\n", name, rows, cols);
		parse_error_flag = 1;
		return -1;
	}
	if(cJSON_GetArraySize(item)!=rows){
		fprintf(stderr, "ERROR: parsing json object: %s should be a matrix with %d rows and %d columns\n", name, rows, cols);
		parse_error_flag = 1;
		return -1;
	}

	for(int i=0; i<rows; i++){
		cJSON* row =cJSON_GetArrayItem(item, i);
		if(cJSON_GetArraySize(row)!=cols){
			fprintf(stderr, "ERROR: parsing json object: %s should be a matrix with %d rows and %d columns\n", name, rows, cols);
			parse_error_flag = 1;
			return -1;
		}
		for(int j=0; j<cols; j++){
			val[(i*cols)+j]=cJSON_GetArrayItem(row, j)->valuedouble;
		}
	}

	return 0;
}


int json_fetch_fixed_matrix_with_default(cJSON* obj, const char* name, double* val, int rows, int cols, double* default_val)
{
	cJSON* item = cJSON_GetObjectItem(obj, name);
	if(item==NULL){
		item = cJSON_CreateArray();
		for(int i=0; i<rows; i++) cJSON_AddItemToArray(item,cJSON_CreateDoubleArray(&default_val[i*cols],cols));
		cJSON_AddItemToObject(obj, name, item);
		modified_flag = 1;
	}
	return json_fetch_fixed_matrix(item, NULL, val, rows, cols);
}


cJSON* json_fetch_object(cJSON* obj, const char* name)
{
	cJSON* item;
	// The user can provide the cjson pointer to the exact item to be fetched or
	// to an object that contains the item by name. Make the choice here:
	if(name==NULL) item = obj;	// look at the current cjson object itself
	else{						// otherwise look for the item by name
		item = cJSON_GetObjectItem(obj, name);
		if(item==NULL){
			fprintf(stderr, "ERROR: object missing %s\n", name);
			parse_error_flag = 1;
			return NULL;
		}
	}

	if(!cJSON_IsObject(item)){
		fprintf(stderr, "ERROR: parsing json object: %s should be a child object\n", name);
		parse_error_flag = 1;
		return NULL;
	}
	return item;
}


cJSON* json_fetch_object_and_add_if_missing(cJSON* obj, const char* name)
{
	cJSON* item = cJSON_GetObjectItem(obj, name);
	if(item==NULL){
		item = cJSON_CreateObject();
		if(!cJSON_AddItemToObject(obj, name, item)){
			fprintf(stderr, "ERROR: could not add %s to JSON object\n", name);
			parse_error_flag = 1;
			return NULL;
		}
		modified_flag = 1;
	}
	if(!cJSON_IsObject(item)){
		fprintf(stderr, "ERROR: parsing json object: %s should be a child object\n", name);
		parse_error_flag = 1;
		return NULL;
	}
	return item;
}


cJSON* json_fetch_array(cJSON* obj, const char* name, int* length)
{
	cJSON* item;
	// The user can provide the cjson pointer to the exact item to be fetched or
	// to an object that contains the item by name. Make the choice here:
	if(name==NULL) item = obj;	// look at the current cjson object itself
	else{						// otherwise look for the item by name
		item = cJSON_GetObjectItem(obj, name);
		if(item==NULL){
			fprintf(stderr, "ERROR: object missing %s\n", name);
			parse_error_flag = 1;
			return NULL;
		}
	}

	if(!cJSON_IsArray(item)){
		fprintf(stderr, "ERROR: parsing json object: %s should be an array\n", name);
		parse_error_flag = 1;
		return NULL;
	}
	*length = cJSON_GetArraySize(item);
	return item;
}


cJSON* json_fetch_array_and_add_if_missing(cJSON* obj, const char* name, int* length)
{
	cJSON* item = cJSON_GetObjectItem(obj, name);
	if(item==NULL){
		item = cJSON_CreateArray();
		if(!cJSON_AddItemToObject(obj, name, item)){
			fprintf(stderr, "ERROR: could not add %s to JSON object\n", name);
			parse_error_flag = 1;
			return NULL;
		}
		modified_flag = 1;
	}
	return json_fetch_array(item, NULL, length);
}


cJSON* json_fetch_array_of_objects(cJSON* obj, const char* name, int* length)
{
	cJSON* item;
	// The user can provide the cjson pointer to the exact item to be fetched or
	// to an object that contains the item by name. Make the choice here:
	if(name==NULL) item = obj;	// look at the current cjson object itself
	else{						// otherwise look for the item by name
		item = cJSON_GetObjectItem(obj, name);
		if(item==NULL){
			fprintf(stderr, "ERROR: object missing %s\n", name);
			parse_error_flag = 1;
			return NULL;
		}
	}

	if(!cJSON_IsArray(item)){
		fprintf(stderr, "ERROR: parsing json object: %s should be an array of objects\n", name);
		parse_error_flag = 1;
		return NULL;
	}
	int len = cJSON_GetArraySize(item);
	for(int i=0;i<len;i++){
		if(!cJSON_IsObject(cJSON_GetArrayItem(item, i))){
			fprintf(stderr, "ERROR: parsing json object: %s should be an array of objects\n", name);
			parse_error_flag = 1;
			return NULL;
		}
	}
	*length = len;
	return item;
}


cJSON* json_fetch_array_of_objects_and_add_if_missing(cJSON* obj, const char* name, int* length)
{
	cJSON* item = cJSON_GetObjectItem(obj, name);
	if(item==NULL){
		item = cJSON_CreateArray();
		cJSON* new_object = cJSON_CreateObject();
		cJSON_AddItemToObject(item, name, new_object);
		if(!cJSON_AddItemToObject(obj, name, item)){
			fprintf(stderr, "ERROR: could not add %s to JSON object\n", name);
			parse_error_flag = 1;
			return NULL;
		}
		modified_flag = 1;
	}

	return json_fetch_array_of_objects(item, NULL, length);
}





int json_remove_if_present(cJSON* obj, const char* name)
{
	if(cJSON_HasObjectItem(obj, name)){
		cJSON_DeleteItemFromObject(obj, name);
		modified_flag = 1;
		return 1;
	}
	return 0;
}
