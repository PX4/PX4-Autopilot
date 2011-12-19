program prices(input, output);
const
   MaxBuffer = 256;
   BASE = 'http://www.irietools.com/';
var
   buffer : string[MaxBuffer];
   NewLocation : string;

   procedure Init;
   begin
      NewLocation := BASE
   end;

   procedure GenerateHTTPHeader;
   begin
      writeln('Content-type: text/html');
      writeln;
   end;

   procedure GetCGIData;
   var
      RequestMethod : string;

      procedure GetRequest;
      begin (* GetRequest *)
         buffer := getenv('QUERY_STRING')
      end; (* GetRequest *)

      procedure PostRequest;
      var
         len, i : 0..maxint;
         err : integer;
         ContentLength : string;
         c : char;
      begin (* PostRequest *)
         buffer := '';
         ContentLength := getenv('CONTENT_LENGTH');
         if ContentLength <> '' then
            val(ContentLength, len, err)
         else
            len := 0;
         if len <= MaxBuffer then
            for i := 1 to len do
               begin
                  read(c);
                  buffer := buffer + c
               end
      end; (* PostRequest *)

   begin (* GetCGIData *)
      RequestMethod := getenv('REQUEST_METHOD');
      if RequestMethod = 'GET' then
         GetRequest
      else
         PostRequest
   end; (* GetCGIData *)

   procedure ProcessCGIData;
   var
      i, num, p : integer;
      EncodedVariable, DecodedVariable, name, value : string;

      procedure ProcessNameValuePair(var name, value : string);
      begin
         if (name = 'lstnavigation') or (name = 'navigation') or (name = 'goto') then
            begin
               if value <> '[none]' then
                  if lowercase(copy(value, 1, 5)) = 'http:' then
                     NewLocation := value
                  else
                     NewLocation := BASE + value
            end
         else
            ; (* do nothing we have an undefined form element *)            
      end;

   begin (* ProcessCGIData *)
      num := CountWords(buffer, '&');
      for i := 1 to num do
         begin
            EncodedVariable := CopyWord(buffer, i, '&');
            DecodedVariable := URLDecode(EncodedVariable);
            p := pos('=', DecodedVariable);
            if p > 0 then
               begin
                  name := lowercase(trim(copy(DecodedVariable, 1, p-1)));
                  value := lowercase(trim(copy(DecodedVariable, p+1)));
                  ProcessNameValuePair(name, value);
               end
         end
   end; (* ProcessCGIData *)

   procedure GenerateResponse;

      procedure GenerateHTMLHeader;
      begin
         writeln('<html>');
         writeln('<head>');
         writeln('<meta name="Description" content="Redirect New Location">');

         writeln('<meta http-equiv="Refresh" content="0;URL=', NewLocation, '">');

         writeln('<title>Redirect to New Location</title>');
         writeln('</head>');
      end;

      procedure GenerateHTMLFooter;
      begin
         writeln('<hr>');
         writeln('<p>');
         writeln('Redirect 1.0 Copyright &copy; 1999-2001, Stuart King<br>');
         writeln('Home page <a href="http://www.irietools.com/">www.irietools.com</a>');
         writeln('</p>');
         writeln('</body>');
         writeln('</html>');
      end;

   begin (* GenerateResponse *)
      GenerateHTMLHeader;
      writeln('<body bgcolor="#FFE8E8">');
      writeln('<p>You should be automatically taken to the next page.</p>');
      writeln('<p>However if your browser does not support redirection ');
      writeln('click <a href="', NewLocation, '">here</a></p>');
      GenerateHTMLFooter;
   end; (* GenerateResponse *)

begin
   GenerateHTTPHeader;

   Init;

   GetCGIData;

   ProcessCGIData;

   GenerateResponse;
end.
