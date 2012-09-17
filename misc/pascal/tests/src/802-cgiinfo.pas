program info(output);

procedure WriteHeader;
begin
writeln('Content-type: text/html');
writeln;
writeln('<html>');
writeln('<head>');
writeln('<title>Irie Pascal sample CGI application</title>');
writeln('<h1>CGI environment variables.</h1>');
writeln('</head>')
end;

procedure WriteBody;

procedure DisplayEnvVar(name : string);
var
value : string;
begin
value := getenv(name);
writeln(name, ' = ', value, '<br>')
end;

begin
writeln('<body>');
DisplayEnvVar('HTTP_ACCEPT');
DisplayEnvVar('HTTP_ACCEPT_ENCODING');
DisplayEnvVar('HTTP_ACCEPT_LANGUAGE');
DisplayEnvVar('HTTP_AUTHORIZATION');
DisplayEnvVar('HTTP_CHARGE_TO');
DisplayEnvVar('HTTP_FROM');
DisplayEnvVar('HTTP_IF_MODIFIED_SINCE');
DisplayEnvVar('HTTP_PRAGMA');
DisplayEnvVar('HTTP_REFERER');
DisplayEnvVar('HTTP_USER_AGENT');
writeln('<hr>');
DisplayEnvVar('AUTH_TYPE');
DisplayEnvVar('CONTENT_LENGTH');
DisplayEnvVar('CONTENT_TYPE');
DisplayEnvVar('GATEWAY_INTERFACE');
DisplayEnvVar('PATH_INFO');
DisplayEnvVar('PATH_TRANSLATED');
DisplayEnvVar('QUERY_STRING');
DisplayEnvVar('REMOTE_ADDR');
DisplayEnvVar('REMOTE_HOST');
DisplayEnvVar('REMOTE_IDENT');
DisplayEnvVar('REMOTE_USER');
DisplayEnvVar('REQUEST_METHOD');
DisplayEnvVar('SCRIPT_NAME');
DisplayEnvVar('SERVER_NAME');
DisplayEnvVar('SERVER_PORT');
DisplayEnvVar('SERVER_PROTOCOL');
DisplayEnvVar('SERVER_SOFTWARE');
writeln('</body>')
end;

procedure WriteFooter;
begin
writeln('</html>')
end;

begin
WriteHeader;
WriteBody;
WriteFooter
end.

