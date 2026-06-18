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

#include <modal_json.h>


int main(int argc, char **argv)
{
	if(argc!=2){
		fprintf(stderr, "please provide a yaml file path to parse\n");
		return -1;
	}

	// parse and error check
	printf("parsing file: %s\n", argv[1]);
	cJSON* json = json_from_yaml(argv[1]);
	if(json==NULL) return -1;

	// print the result
	printf("\n%s\n", cJSON_Print(json));

	// cleanup
	cJSON_Delete(json);
	return 0;
}
