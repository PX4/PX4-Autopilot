/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file zenoh_config.cpp
 *
 * Zenoh configuration backend
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#include "zenoh_config.hpp"

#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

#include <uORB/topics/uORBTopics.hpp>


const char *default_net_config = Z_CONFIG_MODE_DEFAULT;
const char *default_pub_config = "";
const char *default_sub_config = ""; //TODO maybe use YAML


Zenoh_Config::Zenoh_Config()
{
	bool correct_config = true;
	DIR *dir = opendir(ZENOH_SD_ROOT_PATH);
	fp_mapping = NULL;

	if (dir) {
		/* Directory exists. */
		closedir(dir);

		if (access(ZENOH_NET_CONFIG_PATH, F_OK) != 0) {
			correct_config = false;

		} else if (access(ZENOH_PUB_CONFIG_PATH, F_OK) != 0) {
			correct_config = false;

		} else if (access(ZENOH_SUB_CONFIG_PATH, F_OK) != 0) {
			correct_config = false;
		}

	} else {
		/* opendir() failed */
		correct_config = false;
	}

	if (!correct_config) {
		generate_clean_config();
	}
}

Zenoh_Config::~Zenoh_Config()
{
	if (fp_mapping != NULL) {
		fclose(fp_mapping);
	}
}

int Zenoh_Config::AddPubSub(char *topic, char *datatype, const char *filename)
{
	{
		char f_topic[TOPIC_INFO_SIZE];
		char f_type[TOPIC_INFO_SIZE];

		while (getPubSubMapping(f_topic, f_type, filename) > 0) {
			if (strcmp(topic, f_topic) == 0
			    || strcmp(datatype, f_type) == 0) {
				printf("Already mapped to uORB %s -> %s\n", f_type, f_topic);
				return 0;
			}

		}
	}

	for (size_t i = 0; i < orb_topics_count(); i++) {
		const struct orb_metadata *meta = get_orb_meta((ORB_ID)i);

		if (meta != NULL &&
		    strcmp(meta->o_name, datatype) == 0) {
			FILE *fp = fopen(filename, "a");

			if (fp) {
				fprintf(fp, "%s;%s\n", topic, datatype);

			} else {
				return -1;
			}

			fclose(fp);
			return 1;
		}
	}

	printf("%s not found\n", datatype);
	return 0;
}


int Zenoh_Config::SetNetworkConfig(char *mode, char *locator)
{

	FILE *fp = fopen(ZENOH_NET_CONFIG_PATH, "w");

	if (fp) {
		if (locator == 0) {
			fprintf(fp, "%s\n", mode);

		} else {
			fprintf(fp, "%s;%s\n", mode, locator);
		}

	} else {
		return -1;
	}

	fclose(fp);
	return 0;
}

int Zenoh_Config::cli(int argc, char *argv[])
{
	if (argc == 1) {
		dump_config();

	} else if (argc == 3) {
		if (strcmp(argv[1], "net") == 0) {
			SetNetworkConfig(argv[2], 0);
		}

	} else if (argc == 4) {
		if (strcmp(argv[1], "addpublisher") == 0) {
			if (AddPubSub(argv[2], argv[3], ZENOH_PUB_CONFIG_PATH) > 0) {
				printf("Added %s %s to publishers\n", argv[2], argv[3]);

			} else {
				printf("Could not add uORB %s -> %s to publishers\n",  argv[3], argv[2]);
			}

		} else if (strcmp(argv[1], "addsubscriber") == 0) {
			if (AddPubSub(argv[2], argv[3], ZENOH_SUB_CONFIG_PATH) > 0) {
				printf("Added %s -> uORB %s to subscribers\n", argv[2], argv[3]);

			} else {
				printf("Could not add %s -> uORB %s to subscribers\n",  argv[2], argv[3]);
			}

		} else if (strcmp(argv[1], "net") == 0) {
			SetNetworkConfig(argv[2], argv[3]);
		}
	}

	//TODO make CLI to modify configuration now you would have to manually modify the files
	return 0;
}

const char *Zenoh_Config::get_csv_field(char *line, int num)
{
	const char *tok;

	for (
		tok = strtok(line, ";");
		tok && *tok;
		tok = strtok(NULL, ";\n")) {
		if (!--num) {
			return tok;
		}
	}

	return NULL;
}

void Zenoh_Config::getNetworkConfig(char *mode, char *locator)
{
	FILE *fp;
	char buffer[NET_CONFIG_LINE_SIZE];

	fp = fopen(ZENOH_NET_CONFIG_PATH, "r");

	// If file opened successfully, then read the file
	if (fp) {
		fgets(buffer, NET_CONFIG_LINE_SIZE, fp);
		const char *config_locator = get_csv_field(buffer, 2);
		char *config_mode = (char *)get_csv_field(buffer, 1);

		if (config_mode) {
			config_mode[strcspn(config_mode, "\n")] = 0;
			strncpy(mode, config_mode, NET_MODE_SIZE);

		} else {
			mode[0] = 0;
		}

		if (config_locator) {
			strncpy(locator, config_locator, NET_LOCATOR_SIZE);

		} else {
			locator[0] = 0;
		}

	} else {
		printf("Failed to open the file\n");
	}

	//Close the file
	fclose(fp);
}

int Zenoh_Config::getLineCount(const char *filename)
{
	int lines = 0;
	int ch;

	// Open file in write mode
	FILE *fp = fopen(filename, "r");

	while ((ch = fgetc(fp)) != EOF) {
		if (ch == '\n') {
			lines++;
		}
	}

	//Close the file
	fclose(fp);

	return lines;
}

// Very rudamentary here but we've to wait for a more advanced param system
int Zenoh_Config::getPubSubMapping(char *topic, char *type, const char *filename)
{
	char buffer[MAX_LINE_SIZE];

	if (fp_mapping == NULL) {
		fp_mapping = fopen(filename, "r");
	}

	if (fp_mapping) {
		while (fgets(buffer, MAX_LINE_SIZE, fp_mapping) != NULL) {
			if (buffer[0] != '\n') {
				const char *config_type = get_csv_field(buffer, 2);
				const char *config_topic = get_csv_field(buffer, 1);

				strncpy(type, config_type, TOPIC_INFO_SIZE);
				strncpy(topic, config_topic, TOPIC_INFO_SIZE);
				return 1;
			}

		}

	} else {
		printf("Failed to open the file\n");
		return -1;
	}

	//Close the file
	fclose(fp_mapping);
	fp_mapping = NULL;
	return 0;

}


void Zenoh_Config::dump_config()
{
	printf("Network config:\n");
	{
		char mode[NET_MODE_SIZE];
		char locator[NET_LOCATOR_SIZE];
		getNetworkConfig(mode, locator);

		printf("Mode: %s\n", mode);

		if (locator[0] == 0) {
			printf("Locator: scout\n");

		} else {
			printf("Locator: %s\n", locator);
		}

		printf("\n");
	}

	{
		char topic[TOPIC_INFO_SIZE];
		char type[TOPIC_INFO_SIZE];

		printf("Publisher config:\n");

		while (getPubSubMapping(topic, type, ZENOH_PUB_CONFIG_PATH) > 0) {
			printf("Topic: %s\n", topic);
			printf("Type: %s\n", type);
		}

		printf("\nSubscriber config:\n");

		while (getPubSubMapping(topic, type, ZENOH_SUB_CONFIG_PATH) > 0) {
			printf("Topic: %s\n", topic);
			printf("Type: %s\n", type);
		}
	}
}


void Zenoh_Config::generate_clean_config()
{
	printf("Generate clean\n");
	FILE *fp;

	DIR *dir = opendir(ZENOH_SD_ROOT_PATH);

	if (dir) {
		printf("Zenoh directory exists\n");

	} else {
		/* Create zenoh dir. */
		if (mkdir(ZENOH_SD_ROOT_PATH, 0700) < 0) {
			printf("Failed to create Zenoh directory\n");
			return;
		}

	}

	if (access(ZENOH_NET_CONFIG_PATH, F_OK) != 0) {
		// Open file in write mode
		fp = fopen(ZENOH_NET_CONFIG_PATH, "w");

		// If file opened successfully, then write the string to file
		if (fp) {
			fputs(default_net_config, fp);

		} else {
			printf("Failed to open the file\n");
			return;
		}

		//Close the file
		fclose(fp);
	}

	if (access(ZENOH_PUB_CONFIG_PATH, F_OK) != 0) {
		// Open file in write mode
		fp = fopen(ZENOH_PUB_CONFIG_PATH, "w");

		// If file opened successfully, then write the string to file
		if (fp) {
			fputs(default_pub_config, fp);

		} else {
			printf("Failed to open the file\n");
			return;
		}

		//Close the file
		fclose(fp);
	}

	if (access(ZENOH_SUB_CONFIG_PATH, F_OK) != 0) {
		// Open file in write mode
		fp = fopen(ZENOH_SUB_CONFIG_PATH, "w");

		// If file opened successfully, then write the string to file
		if (fp) {
			fputs(default_sub_config, fp);

		} else {
			printf("Failed to open the file\n");
			return;
		}

		//Close the file
		fclose(fp);
	}
}
