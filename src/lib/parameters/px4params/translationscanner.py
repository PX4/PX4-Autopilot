import xml.etree.ElementTree as ET
#import os 
from os import listdir
from os.path import isfile, join, dirname, realpath, exists


def GetTranslations(filename=''):
    lang_translations = dict()
    # Get path to translation path

    dir_path = dirname(realpath(__file__))
    translation_path = dir_path.rsplit('/',1)[0]+'/translations/'
    if not exists(translation_path):
        #print("DEBUG: Translation direcory not found: %s" % translation_path)
        return None
    translation_files = [f for f in listdir(translation_path) if isfile(join(translation_path, f))]

    # Get english dict
    for filename in translation_files:
        if filename=='parameter_strings.xml':
            english_root = ET.parse(translation_path+filename)
            lang_dict = dict()
            for entry in english_root.findall("./entry"):
                lang_dict[entry.attrib['key']]=entry.text
                #print("DEBUG: %s " % entry.attrib['key'])
                #print("DEBUG: %s " % entry.text)
                lang_translations['en'] = lang_dict
            break

    # Return if no English version found (need for keys)
    if len(lang_translations)==0:
        return None

    for filename in translation_files:
        #print("DEBUG: %s" % filename)
        if filename=='parameter_strings.xml':
            continue
        else:
            root = ET.parse(translation_path+filename)
            lang_dict = dict()
            lang_key=filename.rsplit('-',1)[-1].rsplit('.',1)[0]
            #print('DEBUG:lang_key: %s' % lang_key)
            for entry in root.findall("./entry"):
                key = entry.attrib['key']
                entry_text = entry.text
                # Only add string if it has changed from English
                if key in lang_translations['en'] and not lang_translations['en'][key] == entry_text:
                    lang_dict[key]=entry_text
        lang_translations[lang_key] = lang_dict
    return lang_translations
