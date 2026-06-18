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
#include <stdlib.h>

#include <modal_json.h>

#define NAMESIZE 128


#define TYPE_NUMBER 1
#define TYPE_STRING 2
#define TYPE_ARRAY  3



static void trim_string(char str[])
{
	int i,j;
	i = 0;
	while(i<(int)strlen(str)){
		if(str[i]==' ' || str[i]=='"'){
			for (j=i; j<(int)strlen(str); j++)
				str[j]=str[j+1];
		} else i++;
	}

	return;
}


// this function is just a first pass, it needs robustifying and testing
cJSON* json_from_yaml(const char* filename)
{
	int i;

	// file reading vars
	FILE * fp;
	size_t lim = 1024;
	size_t lim2 = 1024;
	char* line = (char*)malloc(lim); // hold each line from file
	char* line2 = (char*)malloc(lim); // combination of multiple wrapped lines
	int len = 0;

	// line parsing vars
	char name[NAMESIZE];

	cJSON* top_level_json = cJSON_CreateObject();
	if(top_level_json == NULL){
		fprintf(stderr, "ERROR: in %s, failed to make new cJSON object\n", __FUNCTION__);
		return NULL;
	}
	// pointer to wherever our current parent is while parsing
	// start by pointing to the top level json object
	cJSON* current_parent = top_level_json;

	// open file for reading
	fp = fopen(filename, "r");
	if(fp == NULL){
		fprintf(stderr, "ERROR in %s, failed to open file\n", __FUNCTION__);
		return NULL;
	}

	while((len = getline(&line, &lim, fp)) != -1) {

		// skip the header lines
		if(line[0]=='%' || line[0]=='-') continue;

		// trim off comments
		for(i=0; i<len; i++){
			if(line[i]=='!' || line[i]=='#'){
				line[i]=0;
				len = i;
				break;
			}
		}

		// trim trailing spaces and newlines
		for(i=(len-1); i>=0; i--){
			if(line[i]==' ' || line[i]=='\t' || line[i]=='\n'){
				line[i] = 0;
				len = i-1;
			}
			else break;
		}

		// count spaces at the beginning of the line
		int spaces = 0;
		for(spaces=0;spaces<len;spaces++){
			if(line[spaces]!=' ') break;
		}
		if(spaces==0) current_parent=top_level_json;

		// if line ends in a comma, keep reading in the next lines
		while(line[len]==','){
			int next_len = getline(&line2, &lim2, fp);
			if(next_len==-1){
				fprintf(stderr, "ERROR in %s, failed to read line after comma\n", __FUNCTION__);
			}
			len += next_len;
			if(len>(int)lim){
				fprintf(stderr, "ERROR in %s, line too long\n", __FUNCTION__);
				return NULL;
			}
			strcat(line, line2);
			if(line[len]=='\n'){
				line[len]=0;
				len--;
			}
		}


		// fetch name from beginning of line
		int namelen=0;
		memset(name, 0, NAMESIZE);
		for(i=spaces;i<len;i++){
			if(line[i]==':'){
				name[namelen]=0;
				break;
			}
			else{
				name[namelen]=line[i];
				namelen++;
			}
		}
		if(i==(len-1)){
			fprintf(stderr, "ERROR in %s, failed to find colon\n", __FUNCTION__);
			return NULL;
		}

		// save the point in the string where the data starts
		char* data = &line[i+1];

		// if line ends in a colon we have a new object
		if(line[len]==':'){
			//printf("new obj: %s\n", name);
			cJSON* new_obj = cJSON_CreateObject();
			cJSON_AddItemToObject(top_level_json, name, new_obj);
			current_parent = new_obj;
			continue;
		}

		// figure out if we have a number, string, or array
		int contains_digit = 0;
		int type = 0;
		for(i=0;data[i]!=0;i++){
			if(data[i]=='['){
				type = TYPE_ARRAY;
				break;
			}
			else if(data[i]=='"'){
				type = TYPE_STRING;
				break;
			}
			else if(data[i]>='0' && data[i]<='9'){
				contains_digit = 1;
			}
		}
		if(type==0){
			if(contains_digit) type = TYPE_NUMBER;
			else type = TYPE_STRING;
		}


		// parse the data for each type
		if(type==TYPE_NUMBER){
			cJSON_AddNumberToObject(current_parent, name, atof(data));
		}
		else if(type==TYPE_STRING){
			trim_string(data);
			cJSON_AddStringToObject(current_parent, name, data);
		}
		else if(type==TYPE_ARRAY){

			int array_items = 0;
			int isnumstart = 0;
			double double_array[128];

			for(int pos=0; pos<(int)strlen(data); pos++){
				if(data[pos]==']') break;
				if(isnumstart){
					double_array[array_items] = atof(&data[pos]);
					array_items++;
					if(array_items>=128){
						fprintf(stderr, "ERROR in %s, array is too long!\n", __FUNCTION__);
						return NULL;
					}
					isnumstart=0;
				}
				else if(data[pos]==',' || data[pos]=='[') isnumstart = 1;
			}
			cJSON_AddItemToObject(current_parent, name, cJSON_CreateDoubleArray(double_array, array_items));
		}
		else{
			fprintf(stderr, "ERROR in %s, unknown type for %s\n", __FUNCTION__, name);
			return NULL;
		}
	}

	fclose(fp);
	if(line) free(line);
	if(line2) free(line2);

	return top_level_json;
}
