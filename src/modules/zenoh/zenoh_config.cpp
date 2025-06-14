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
#include <ctype.h>

#include <uORB/topics/uORBTopics.hpp>


const char *default_net_config = Z_CONFIG_MODE_DEFAULT ";" CONFIG_ZENOH_DEFAULT_LOCATOR;

// Default config generated from default_topics.c.em and dds_topics.yaml
extern const char *default_pub_config;
extern const char *default_sub_config;


Zenoh_Config::Zenoh_Config()
{
	bool correct_config = true;
	DIR *dir = opendir(ZENOH_ROOT_PATH);
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

int Zenoh_Config::AddPubSub(char *topic, char *datatype, int instance_no, const char *filename)
{
	{
		char f_topic[TOPIC_INFO_SIZE];
		char f_type[TOPIC_INFO_SIZE];
		int f_new_instance;

		while (getPubSubMapping(f_topic, f_type, &f_new_instance, filename) > 0) {
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
				fprintf(fp, "%s;%s;%d\n", topic, datatype, instance_no);

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


int Zenoh_Config::DeletePubSub(char *topic, const char *filename)
{
	if (!filename || !topic) {
		return -1;
	}

	FILE *file = fopen(filename, "r");

	if (!file) {
		return -1;
	}

	// Create a temporary file for writing
	char temp_filename[256];
	snprintf(temp_filename, sizeof(temp_filename), "%s.tmp", filename);
	FILE *temp_file = fopen(temp_filename, "w");

	if (!temp_file) {
		fclose(file);
		return -1;
	}

	char line[TOPIC_INFO_SIZE];
	char line_copy[TOPIC_INFO_SIZE];
	const char *fields[1];  // We only need the topic
	int found = 0;

	while (fgets(line, sizeof(line), file)) {
		// Remove newline if present
		size_t len = strlen(line);

		if (len > 0 && line[len - 1] == '\n') {
			line[len - 1] = '\0';
		}

		// Make a copy of the line before parsing since parse_csv_line will replace ;s with \0s
		strncpy(line_copy, line, sizeof(line_copy) - 1);
		line_copy[sizeof(line_copy) - 1] = '\0';
		int num_fields = parse_csv_line(line_copy, fields, 1);

		// If the topic doesn't match the topic, write the line to temp file
		if (num_fields == 0 || strcmp(fields[0], topic) != 0) {
			fprintf(temp_file, "%s\n", line);

		} else {
			found = 1;
		}
	}

	fclose(file);
	fclose(temp_file);

	// replace the original file if deletion was successful
	if (found) {
		if (remove(filename) != 0) {
			remove(temp_filename);
			return -1;
		}

		if (rename(temp_filename, filename) != 0) {
			remove(temp_filename);
			return -1;
		}

	} else {
		// Otherwise, if no line was deleted, remove the temp file
		remove(temp_filename);
	}

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

	} else if (argc == 2) {
		if (strcmp(argv[1], "delete") == 0) {
			printf("The type of resource to be deleted (publisher/subscriber) must be specified.\n");

		} else if (strcmp(argv[1], "add") == 0) {
			printf("The type of resource to be added (publisher/subscriber) must be specified.\n");

		} else {
			printf("Unrecognized command\n");
		}

	} else if (argc == 3) {
		if (strcmp(argv[1], "net") == 0) {
			SetNetworkConfig(argv[2], 0);

		} else if (strcmp(argv[1], "delete") == 0) {
			printf("The name of the resource to be deleted needs to be specified.\n");

		} else if (strcmp(argv[1], "add") == 0) {
			printf("The name of the Zenoh/uOrb topic pair to be linked needs to be specified.\n");

		} else {
			printf("Unrecognized command\n");
		}

	} else if (argc == 4) {
		if (strcmp(argv[1], "net") == 0) {
			SetNetworkConfig(argv[2], argv[3]);

		} else if (strcmp(argv[1], "delete") == 0) {
			if (strcmp(argv[2], "publisher") == 0) {
				int res = DeletePubSub(argv[3], ZENOH_PUB_CONFIG_PATH);

				if (res < 0) {
					printf("Could not delete publisher topic %s\n", argv[3]);
				}

			} else if (strcmp(argv[2], "subscriber") == 0) {
				int res = DeletePubSub(argv[3], ZENOH_SUB_CONFIG_PATH);

				if (res < 0) {
					printf("Could not delete subscriber topic %s\n", argv[3]);
				}

			} else {
				printf("Unrecognized command\n");
			}

		} else {
			printf("Unrecognized command\n");
		}

	} else if (argc >= 5) {
		if (strcmp(argv[1], "add") == 0) {
			if (strcmp(argv[2], "publisher") == 0) {
				int instance = 0;

				if (argc == 6) {
					if (sscanf(argv[5], "%d", &instance) != 1 || instance < 0) {
						printf("Invalid instance %s (must be an integer, 0 for the default instance or a specific instance's index)\n",
						       argv[5]);
						return 0;
					}
				}

				if (AddPubSub(argv[3], argv[4], instance, ZENOH_PUB_CONFIG_PATH) > 0) {
					printf("Added %s %s to publishers (instance %d)\n", argv[3], argv[4], instance);

				} else {
					printf("Could not add uORB %s:%d -> %s to publishers\n",  argv[3], instance, argv[4]);
				}

			} else if (strcmp(argv[2], "subscriber") == 0) {
				int instance = 0;

				if (argc == 6) {
					if (sscanf(argv[5], "%d", &instance) != 1 || instance == 0 || instance == -1) {
						printf("Invalid instance %s (must be an integer, 0 for the default instance or -1 for a new uOrb instance)\n", argv[5]);
						return 0;
					}
				}

				if (AddPubSub(argv[3], argv[4], instance, ZENOH_SUB_CONFIG_PATH) > 0) {
					printf("Added %s -> uORB %s:%d to subscribers\n", argv[3], argv[4], instance);

				} else {
					printf("Could not add %s -> uORB %s to subscribers\n",  argv[3], argv[4]);
				}

			} else {
				printf("Unrecognized command\n");
			}

		} else {
			printf("Unrecognized command\n");
		}
	} // doesn't need an else because negative argc would be... weird

	//TODO make CLI to modify configuration now you would have to manually modify the files
	return 0;
}

int Zenoh_Config::parse_csv_line(char *line, const char **fields, int max_fields)
{
	if (!line || !fields || max_fields <= 0) {
		return 0;
	}

	int field_count = 0;
	char *token = strtok(line, ";");

	while (token && field_count < max_fields) {
		// Trim leading whitespace
		while (isspace((unsigned char)*token)) { token++; }

		// Trim trailing whitespace
		char *end = token + strlen(token) - 1;

		while (end > token && isspace((unsigned char)*end)) {
			*end = '\0';
			end--;
		}

		fields[field_count] = token;
		field_count++;
		token = strtok(NULL, ";\n\r");
	}

	return field_count;
}
void Zenoh_Config::getNetworkConfig(char *mode, char *locator)
{
	FILE *fp;
	char buffer[NET_CONFIG_LINE_SIZE];

	fp = fopen(ZENOH_NET_CONFIG_PATH, "r");

	// If file opened successfully, then read the file
	if (fp) {
		fgets(buffer, NET_CONFIG_LINE_SIZE, fp);
		const char *fields[2];
		int nfields = parse_csv_line(buffer, fields, 2);

		if (nfields < 1) {
			PX4_ERR("Invalid Zenoh net config file (must contain the mode and optional locator separated by a ;).");
			fclose(fp);
			return;
		}

		char *config_mode = (char *)fields[0];

		if (config_mode) {
			config_mode[strcspn(config_mode, "\n")] = 0;
			strncpy(mode, config_mode, NET_MODE_SIZE);

		} else {
			mode[0] = 0;
		}

		if (nfields >= 2) {
			const char *config_locator = fields[1];
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

int Zenoh_Config::closePubSubMapping()
{
	if (fp_mapping != NULL) {
		//Close the file
		fclose(fp_mapping);
		fp_mapping = NULL;
		return 0;
	}

	return 0;
}


// Very rudamentary here but we've to wait for a more advanced param system
int Zenoh_Config::getPubSubMapping(char *topic, char *type, int *instance, const char *filename)
{
	char buffer[MAX_LINE_SIZE];

	if (fp_mapping == NULL) {
		fp_mapping = fopen(filename, "r");
	}

	if (fp_mapping) {
		while (fgets(buffer, MAX_LINE_SIZE, fp_mapping) != NULL) {

			if (buffer[0] != '\n') {
				const char *fields[3];
				int nfields = parse_csv_line(buffer, fields, 3);


				if (nfields >= 2) {
					if (nfields == 3) {
						if (sscanf(fields[2], "%d", instance) != 1) {
							PX4_WARN("Malformed zenoh config instance %s (instance field should be an integer following the type)\n", fields[2]);
							return -1;
						}

					} else {
						*instance = -1;
					}

					strncpy(type, fields[1], TOPIC_INFO_SIZE);
					strncpy(topic, fields[0], TOPIC_INFO_SIZE);
					return 1;

				} else {
					return -1;
				}
			}

		}

	} else {
		printf("Failed to open the file\n");
		return -1;
	}

	//Close the file
	return closePubSubMapping();

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
		int instance_no;

		printf("Publisher config:\n");

		while (getPubSubMapping(topic, type, &instance_no, ZENOH_PUB_CONFIG_PATH) > 0) {
			printf("Topic: %s\n", topic);
			printf("Type: %s\n", type);
			printf("Instance: %d\n", instance_no);
		}

		printf("\nSubscriber config:\n");

		while (getPubSubMapping(topic, type, &instance_no, ZENOH_SUB_CONFIG_PATH) > 0) {
			printf("Topic: %s\n", topic);
			printf("Type: %s\n", type);
			printf("Instance: %d\n", instance_no);
		}
	}
}


void Zenoh_Config::generate_clean_config()
{
	printf("Generate clean\n");
	FILE *fp;

	DIR *dir = opendir(ZENOH_ROOT_PATH);

	if (dir) {
		printf("Zenoh directory exists\n");

	} else {
		/* Create zenoh dir. */
		if (mkdir(ZENOH_ROOT_PATH, 0700) < 0) {
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
