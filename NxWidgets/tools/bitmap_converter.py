#!/usr/bin/env python

'''This script converts from any image type supported by
Python imaging library to the RLE-encoded format used by
NxWidgets.
'''

from PIL import Image

def get_palette(img, maxcolors = 255):
  '''Returns a list of colors. If there are too many colors in the image,
  the least used are removed.
  '''
  img = img.convert("RGB")
  colors = img.getcolors(65536)
  colors.sort(key = lambda c: -c[0])
  return [c[1] for c in colors[:maxcolors]]

def write_palette(outfile, palette):
  '''Write the palette (normal and hilight) to the output file.'''
  
  outfile.write('static const NXWidgets::nxwidget_pixel_t palette[BITMAP_PALETTESIZE] =\n');
  outfile.write('{\n')
  
  for i in range(0, len(palette), 4):
    outfile.write('  ');
    for r, g, b in palette[i:i+4]:
      outfile.write('MKRGB(%3d,%3d,%3d), ' % (r, g, b))
    outfile.write('\n');
  
  outfile.write('};\n\n')
  
  outfile.write('static const NXWidgets::nxwidget_pixel_t hilight_palette[BITMAP_PALETTESIZE] =\n');
  outfile.write('{\n')
  
  for i in range(0, len(palette), 4):
    outfile.write('  ');
    for r, g, b in palette[i:i+4]:
      r = min(255, r + 50)
      g = min(255, g + 50)
      b = min(255, b + 50)
      outfile.write('MKRGB(%3d,%3d,%3d), ' % (r, g, b))
    outfile.write('\n');
  
  outfile.write('};\n\n')
  

def quantize(color, palette):
  '''Return the color index to closest match in the palette.'''
  try:
    return palette.index(color)
  except ValueError:
    # No exact match, search for the closest
    def distance(color2):
      return sum([(a - b)**2 for a, b in zip(color, color2)])
    
    return palette.index(min(palette, key = distance));

def encode_row(img, palette, y):
  '''RLE-encode one row of image data.'''
  entries = []
  color = None
  repeats = 0
  
  for x in range(0, img.size[0]):
    c = quantize(img.getpixel((x, y)), palette)
    if c == color:
      repeats += 1
    else:
      if color is not None:
        entries.append((repeats, color))
      
      repeats = 1
      color = c

  if color is not None:
    entries.append((repeats, color))
      
  return entries

def write_image(outfile, img, palette):
  '''Write the image contents to the output file.'''

  outfile.write('static const NXWidgets::SRlePaletteBitmapEntry bitmap[] =\n');
  outfile.write('{\n');
  
  for y in range(0, img.size[1]):
    entries = encode_row(img, palette, y)
    row = ""
    for r, c in entries:
      if len(row) > 60:
        outfile.write('  ' + row + '\n')
        row = ""
      
      row += '{%3d, %3d}, ' % (r, c)
    
    row += ' ' * (73 - len(row))
    outfile.write('  ' + row + '/* Row %d */\n' % y)
  
  outfile.write('};\n\n');

def write_descriptor(outfile, name):
  '''Write the public descriptor structure for the image.'''
  
  outfile.write('extern const struct NXWidgets::SRlePaletteBitmap g_%s =\n' % name)
  outfile.write('{\n')
  outfile.write('  CONFIG_NXWIDGETS_BPP,\n')
  outfile.write('  CONFIG_NXWIDGETS_FMT,\n')
  outfile.write('  BITMAP_PALETTESIZE,\n')
  outfile.write('  BITMAP_WIDTH,\n')
  outfile.write('  BITMAP_HEIGHT,\n')
  outfile.write('  {palette, hilight_palette},\n')
  outfile.write('  bitmap\n')
  outfile.write('};\n')
  
if __name__ == '__main__':
  import sys
  import os.path
  
  if len(sys.argv) != 3:
    print "Usage: bitmap_converter.py source.png output.cxx"
    sys.exit(1)
  
  img = Image.open(sys.argv[1])
  outfile = open(sys.argv[2], 'w')
  palette = get_palette(img)
  
  outfile.write(
'''
/* Automatically NuttX bitmap file. */
/* Generated from %(src)s by bitmap_converter.py. */

#include <nxconfig.hxx>
#include <crlepalettebitmap.hxx>

#define BITMAP_WIDTH %(width)s
#define BITMAP_HEIGHT %(height)s
#define BITMAP_PALETTESIZE %(palettesize)s

''' % {'src': sys.argv[1], 'width': img.size[0], 'height': img.size[1],
       'palettesize': len(palette)}
  )

  name = os.path.splitext(os.path.basename(sys.argv[1]))[0]
  
  write_palette(outfile, palette)
  write_image(outfile, img, palette)
  write_descriptor(outfile, name)
