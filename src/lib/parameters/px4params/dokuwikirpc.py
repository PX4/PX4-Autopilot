try:
    import xmlrpclib
except ImportError:
    import xmlrpc.client as xmlrpclib 

# See https://www.dokuwiki.org/devel:xmlrpc for a list of available functions!
# Usage example:
#     xmlrpc = dokuwikirpc.get_xmlrpc(url, username, password)
#     print(xmlrpc.dokuwiki.getVersion())

def get_xmlrpc(url, username, password):
    #proto, url = url.split("://")
    #url = proto + "://" + username + ":" + password + "@" + url + "/lib/exe/xmlrpc.php"
    url += "/lib/exe/xmlrpc.php?u=" + username + "&p=" + password
    
    return xmlrpclib.ServerProxy(url)
