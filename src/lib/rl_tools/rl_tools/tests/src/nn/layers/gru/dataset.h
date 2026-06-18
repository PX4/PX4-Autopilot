#include <iostream>
#include <fstream>
#include <vector>
#include <string>
//#include <zlib.h>
//#include <cJSON.h>

//bool decompressFile(const std::string& source, std::vector<char>& outBuffer) {
//    gzFile gzFile = gzopen(source.c_str(), "rb");
//    if (!gzFile) {
//        std::cerr << "Error opening file: " << source << std::endl;
//        return false;
//    }
//
//    const size_t bufferSize = 8192;
//    char buffer[bufferSize];
//    int bytesRead = 0;
//
//    while ((bytesRead = gzread(gzFile, buffer, bufferSize)) > 0) {
//        outBuffer.insert(outBuffer.end(), buffer, buffer + bytesRead);
//    }
//
//    if (gzclose(gzFile) != Z_OK) {
//        std::cerr << "Error closing file: " << source << std::endl;
//        return false;
//    }
//
//    // Ensure the buffer is null-terminated for string operations
//    outBuffer.push_back('\0');
//
//    return true;
//}

//// https://www.kaggle.com/datasets/ltcmdrdata/plain-text-wikipedia-202011
//template <typename TI>
//std::string load_dataset_wikipedia_plaintext(std::string data_path){
//    std::vector<char> decompressed_data;
//
//    if(decompressFile(data_path, decompressed_data)) {
//        std::cout << "Loading dataset: " << data_path << std::endl;
//        cJSON* json = cJSON_Parse(decompressed_data.data());
//        if(json == nullptr) {
//            std::cerr << "Error parsing JSON data." << std::endl;
//            return "";
//        }
//        if(!cJSON_IsArray(json)){
//            std::cerr << "Expected an array of objects." << std::endl;
//            return "";
//        }
//        TI size = cJSON_GetArraySize(json);
//        std::string output;
//        for(TI i = 0; i < size; i++){
//            cJSON* obj = cJSON_GetArrayItem(json, i);
//            if(!cJSON_IsObject(obj)){
//                std::cerr << "Expected an object." << std::endl;
//                return "";
//            }
//            cJSON* text = cJSON_GetObjectItem(obj, "text");
//            if(!cJSON_IsString(text)){
//                std::cerr << "Expected a string." << std::endl;
//                return "";
//            }
//            output += "\n";
//            output += cJSON_GetStringValue(text);
//        }
//        cJSON_Delete(json);
//        return output;
//    } else {
//        std::cerr << "Failed to decompress the file.\n";
//        return "";
//    }
//}


//template <typename TI>
//std::string load_dataset_enwik8(std::string data_path){
//    std::vector<char> decompressed_data;
//
//    if(decompressFile(data_path, decompressed_data)) {
//        return std::string(decompressed_data.begin(), decompressed_data.end());
//    } else {
//        std::cerr << "Failed to decompress the file.\n";
//        return "";
//    }
//}

template <typename TI>
std::string load_dataset_enwik8(std::string data_path){
    std::ifstream file(data_path);
    if(!file.is_open()){
        std::cerr << "Failed to open file: " << data_path << std::endl;
        return "";
    }
    std::string output;
    std::string line;
    while(std::getline(file, line)){
        output += line;
    }
    return output;
}

