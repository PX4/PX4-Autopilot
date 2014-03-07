import os
import re
import codecs

class Scanner(object):
    """
    Traverses directory tree, reads all source files, and passes their contents
    to the Parser.
    """

    re_file_extension = re.compile(r'\.([^\.]+)$')

    def ScanDir(self, srcdir, parser):
        """
        Scans provided path and passes all found contents to the parser using
        parser.Parse method.
        """
        extensions = set(parser.GetSupportedExtensions())
        for dirname, dirnames, filenames in os.walk(srcdir):
            for filename in filenames:
                m = self.re_file_extension.search(filename)
                if m:
                    ext = m.group(1)
                    if ext in extensions:
                        path = os.path.join(dirname, filename)
                        self.ScanFile(path, parser)

    def ScanFile(self, path, parser):
        """
        Scans provided file and passes its contents to the parser using
        parser.Parse method.
        """
        with codecs.open(path, 'r', 'utf-8') as f:
            contents = f.read()
        parser.Parse(contents)
