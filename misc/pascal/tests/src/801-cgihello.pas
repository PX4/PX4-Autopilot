program hello(output);

    procedure WriteResponseHeader;
    begin
        writeln('content-type: text/html');
        writeln
    end;

begin
    WriteResponseHeader;
    writeln('<HTML>');
    writeln('<HEAD>');
    writeln('<TITLE>IriePascal Hello World Program</TITLE>');
    writeln('</HEAD>');
    writeln('<BODY>');
    writeln('<BIG> Hello world!!! </BIG>');
    writeln('</BODY>');
    writeln('</HTML>')
end.

