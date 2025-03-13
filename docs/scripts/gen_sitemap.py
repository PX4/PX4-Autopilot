#! /usr/bin/python

"""

"""

#import lxml.etree as ET
#import requests
#from bs4 import BeautifulSoup as bs
import re
import os # for walk, evironment vars
import subprocess #so I can use git to get the modified dates.
import argparse

dir_name='.'
#git log -1 --format="%as" -- .\zh\uavcan\notes.md

include_dirs = set(['en','zh','ko','uk']) #update for new language builds.
exclude_dirs = set(['.vitepress','node_modules']) #update for new language builds.

my_parser = argparse.ArgumentParser(description='Generate sitemap for all markdown files in directory (default to main for output)')
# Add the arguments                      
my_parser.add_argument('-v',
                       '--version',
                       action='store',
                       type=str,
                       #nargs=1,
                       default='main')
my_parser.add_argument('-d',
                       '--date',
                       action='store_true',
                       help='generate date information')
my_parser.add_argument('-o',
                       '--output',
                       action='store',
                       type=str,
                       #nargs=1,
                       default='./.vitepress/dist/')

# Execute the parse_args() method
args = my_parser.parse_args()
build_version = args.version

#Get build version from process env by preference.
BRANCH_NAME = os.getenv('BRANCH_NAME')
if BRANCH_NAME:
    build_version=BRANCH_NAME
    
url_prefix = 'https://docs.px4.io/%s' % build_version

sitemapitems=[]

for subdir, dirs, files in os.walk(dir_name, topdown=True):
    
    if subdir == '.':
        #print("RootFile: %s" % originalfile)
        #Handle a root file.
        continue
    
    # Check if any of the include directories is in the subdir path
    if any(f"/{inc_dir}/" in subdir or f"\\{inc_dir}\\" in subdir for inc_dir in include_dirs):
        pass
        #print(f"SUBDIR: {subdir}")
    else:
        continue

    if any(f"/{ex_dir}/" in subdir or f"\\{ex_dir}\\" in subdir for ex_dir in exclude_dirs):
        continue
        #print(f"SUBDIR Ex: {subdir}")
        

    for file in files:
        #print(f"xxDebug: {file}")
        sitemapitem = dict()
        sitemapitem['changefreq']='daily'
        if not file.endswith('.md'): #only process md files.
           #print(f"Skip: {file} (not md)")
           continue
       
        originalfile=subdir+'\\'+file
        dir_name=subdir[2:].replace('\\','/')
        orig_file_forwardslash=originalfile.replace('\\','/')
        #git log -1 --format="%as" -- .\zh\uavcan\notes.md
        if args.date:
            modified_datestamp = subprocess.run(["git", "log", "-1", '--format="%as"', "--", "%s" % orig_file_forwardslash],capture_output=True).stdout.decode('UTF-8')
            sitemapitem['modified']=modified_datestamp.strip().strip('"')
            #print(f"debugXX: {sitemapitem['modified']}")
        file_name=file[:-3]+'.html'
        if file_name.startswith('README'):
            file_name=''
        if file_name.startswith('index'):
            file_name=''
        url=f"{url_prefix}/{dir_name}/{file_name}"
        sitemapitem['url']=url

        #print("OrigFile: %s" % originalfile)
        #print("dir_name: %s" % dir_name)
        #print("Subdir: %s" % subdir )
        #print("file_name: %s" % file_name)
        #print(sitemapitem['url'])
        
        sitemapitems.append(sitemapitem)
        
        
# Generate the sitemap from the sitemapitems
all_sitemap_item_text = ""
for item in sitemapitems:
   sitemap_item_text=''
   sitemap_item_text+='  <url>\n'
   sitemap_item_text+=f"    <loc>{item['url']}</loc>\n"
   sitemap_item_text+=f"    <changefreq>{item['changefreq']}</changefreq>\n"
   if args.date:
       sitemap_item_text+=f"    <lastmod>{item['modified']}</lastmod>\n"
   sitemap_item_text+='  </url>\n'
   
   all_sitemap_item_text+=sitemap_item_text

sitemaptext = '''<?xml version="1.0" encoding="UTF-8"?>
<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">
%s</urlset>
''' % all_sitemap_item_text

# Write the sitemap to file
outputfile=args.output+'sitemap.xml'
with open(outputfile,"w") as f: 
    f.write(sitemaptext)

print("Sitemap generated to: %s" % outputfile)

#print("BRANCH_NAME: %s" % BRANCH_NAME)
#print("Build version: %s" % build_version)

