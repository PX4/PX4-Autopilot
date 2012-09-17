//***********************************************************************
//CGIMAIL Version 1.0
//Copyright (c) 2001, Stuart King. All rights reserved.
//PURPOSE: This program is a Common Gateway Interface (CGI) application
//         that sends email messages.
//***********************************************************************
program cgimail(input, output);
const
   MaxBuffer = 2400; //Size of the buffer used to store the data from GET and POST requests
   DEFAULT_SENDMAIL = '/usr/lib/sendmail'; //Default fullpathname to the sendmail program
   DEFAULT_SUCCESS_MESSAGE = '<p>Thank you. Your email message has been sent successfully.</p>';
   CONFIG_FILENAME = 'cgimail.cfg'; //name of the configuration file
type
   Buffer = string[MaxBuffer];
var
   strCGIData : Buffer; //Stores the data from the GET or POST request
   strFrom, strTo, strSubject, strBody : Buffer;
   strErrorLink, strSuccessLink : filename;
   strSuccessMessage : Buffer;
   SendMail : filename;

   procedure DisplayError(strMsg : string);
   begin //DisplayError
      writeln('ERROR: ', strMsg);
      halt
   end;  //DisplayError

   //*************************************************************
   //PURPOSE: Returns the directory containing the application
   //NOTES: Used in this particular program to help locate the
   //       configuration file.
   //*************************************************************
   function AppDir : filename;
   var
      fdir : filename;
   begin
      fsplit(paramstr(0), fdir,,);
      AppDir := fdir
   end;

   //*****************************************************************
   //PURPOSE: Returns an output string (strOut) that is generated
   //         from an input string (strIn), by replacing all
   //         occurrences of one string (strOld) with another
   //         string (strNew).
   //NOTE: No error checking is done so it is up to the caller to
   //      ensure that the replacing string does not contain the
   //      string being replaced.
   //*****************************************************************
   function Replace(strIn : Buffer; strOld, strNew : string) : Buffer;
   var
      strOut : Buffer;
      i : 0..MaxBuffer;
   begin
      strOut := strIn;
      i := 1;
      repeat
         i := pos(strOld, strOut, i);
         if i > 0 then
            begin
               delete(strOut, i, length(strOld));
               insert(strNew, strOut, i);
            end
      until i = 0;
      Replace := strOut
   end;

   (***************************************************************
   ** PURPOSE: This procedure converts certain characters that
   **          have a special meaning in HTML documents to
   **          their HTML representation.
   **          The characters converted are < > "
   *)
   function EscapeCharacters(str : Buffer) : Buffer;
   const
      LessThanChar = '<';
      GreaterThanChar = '>';
      QuoteChar = '"';
      HTMLLessThan = '&lt;';
      HTMLGreaterThan = '&gt;';
      HTMLQuote = '&quot;';

   begin
      str := Replace(str, LessThanChar, HTMLLessThan);
      str := Replace(str, GreaterThanChar, HTMLGreaterThan);
      str := Replace(str, QuoteChar, HTMLQuote);
      EscapeCharacters := str
   end;

   procedure Initialize;
   begin //Initialize
      strFrom := '';
      strTo := '';
      strSubject := '';
      strBody := '';
      SendMail := DEFAULT_SENDMAIL;
      strSuccessMessage := DEFAULT_SUCCESS_MESSAGE;
      strErrorLink := '';
      strSuccessLink := '';
   end;  //Initialize

   //****************************************************************
   //PURPOSE: Looks for the configuration file in the application
   //          directory, and if found prcesses the commands inside.
   //****************************************************************
   procedure ReadConfigFile;
   var
      ConfigFile : filename;
      f : text;
      line : string;

      //**********************************************************
      //PURPOSE: Processes a line in the configuration file
      //NOTE: The line is assumed to look like
      //         command=value
      //      where "command" is a valid configuration command
      //        and "value" is a valid configuration value
      //        for example
      //
      //             sendmail=/usr/lib/sendmail
      //
      //**********************************************************
      procedure ProcessConfigLine(line : string);
      const
         SENDMAIL_COMMAND = 'sendmail';
         TO_COMMAND = 'to';
         SUBJECT_COMMAND = 'subject';
         FROM_COMMAND = 'from';
         BODY_COMMAND = 'body';
         ERROR_LINK_COMMAND = 'error_link';
         SUCCESS_LINK_COMMAND = 'success_link';
         SUCCESS_MESSAGE_COMMAND = 'success_message';
      var
         command, value : string;
      begin //ProcessConfigLine
         if CountWords(line, '=') = 2 then
            begin
               command := lowercase(trim(CopyWord(line, 1, '=')));
               value := trim(CopyWord(line, 2, '='));
               if command = SENDMAIL_COMMAND then
                  SendMail := value
               else if command = TO_COMMAND then
                  strTo := value
               else if command = SUBJECT_COMMAND then
                  strSubject := value
               else if command = FROM_COMMAND then
                  strFrom := value
               else if command = BODY_COMMAND then
                  strBody := value
               else if command = ERROR_LINK_COMMAND then
                  strErrorLink := value
               else if command = SUCCESS_LINK_COMMAND then
                  strSuccessLink := value
               else if command = SUCCESS_MESSAGE_COMMAND then
                  strSuccessMessage := value
            end;
      end;  //ProcessConfigLine

   begin //ReadConfigFile
      ConfigFile := AppDir;   //Look for configuration file in the Application directory
      if ConfigFile <> '' then
         begin
            //Ensure that the Application directory ends with a directory seperator
            if ConfigFile[length(ConfigFile)] <> dirsep then
               ConfigFile := ConfigFile + dirsep;
            //Append the name of the configuration file to the application directory
            ConfigFile := ConfigFile + CONFIG_FILENAME;
         end
      else //this shouldn't happen but just in case look for configuration file in current directory.
         ConfigFile := CONFIG_FILENAME;

      traperrors(false);      //Turn off error trapping
      reset(f, ConfigFile);   //Attempt to open configuaration file
      traperrors(true);       //Turn on error trapping
      if getlasterror = 0 then //if the configuration file was successfully opened read it and process it
         begin
            while not eof(f) do
               begin
                  readln(f, line);
                  ProcessConfigLine(line);
               end;
            close(f);
         end;
   end;  //ReadConfigFile

   //***************************************************************
   //PURPOSE: Checks whether the input string (strAddress) is a
   //          valid email address.
   //NOTE: This function does not perform a bullet-proof check
   //       for the validity of the email address. It just ensures
   //       that the address has at least one  '@' and the first '@' is
   //       not at the beginning of the address. This function also
   //       checks to make sure that a '.' follows somewhere after the '@'.
   //***************************************************************
   function IsValidEmailAddress(strAddress : Buffer) : boolean;
   var
      i : integer;
   begin //IsValidEmailAddress
      i := pos('@', strAddress);
      if (i > 1) and (pos('.', strAddress, i+1) > 0) then
         IsValidEmailAddress := true
      else
         IsValidEmailAddress := false;
   end;  //IsValidEmailAddress

   //*********************************************************************
   //PURPOSE: Sends an email message using "sendmail" or the CDONTS object
   //*********************************************************************
   procedure SendEmail(strFrom, strTo, strSubject, strBody : Buffer);

      //********************************************************
      //PURPOSE: Sends and email message using "sendmail"
      //NOTE:
      //The built-in procedure "popen" is used to execute "sendmail" and
      //    open a pipe to the "sendmail" process. The email message is then
      //    written to this pipe and the "sendmail" process reads the email
      //    message and sends it to the recipients.
      //The "-t" option is passed to "sendmail" to cause it to examine the
      //    email message for To:, Cc:, and Bcc: headers which it will then
      //    use to find out who are the intended recipients of the message.
      //The configuration file (cgimail.cfg) can be used to specify where
      //    "sendmail" is located or even that another program be used
      //    instead of "sendmail". See the constant DEFAULT_SENDMAIL at the
      //    top of this program for the default location of "sendmail".
      //If "sendmail" is not in the default location then use a command like
      //
      //          sendmail=pathname
      //
      //    in the configuration file to specify the correct location.
      //You can also use the "sendmail" command in the configuration file
      //    to specify that a program other than "sendmail" be used
      //    to send email (under Unix-like operating systems), as long
      //    as the substitute email program has a similar interface to
      //    "sendmail", (i.e. it can take the -t option, and it will examine
      //    the email message for (at minimum) a To: header).
      //The configuration file (cgimail.cfg) should be placed in the same
      //    directory as this program.
      //********************************************************
      procedure SendEmailWithSendMail;
      const
         SENDMAIL_OPTIONS = '-t'; //Make sendmail examine To:, Cc:, and Bcc: in message
      var
         pipe : text;
      begin //SendEmailWithSendMail
         popen(pipe, SendMail + ' ' + SENDMAIL_OPTIONS, writemode);
         writeln(pipe, 'To: ', strTo);
         if strFrom <> '' then
            writeln(pipe, 'From: ', strFrom);
         if strSubject <> '' then
            writeln(pipe, 'Subject: ', strSubject);
         writeln(pipe);
         if strBody <> '' then
            writeln(pipe, strBody);
         writeln(pipe, '.');
         close(pipe);
      end;  //SendEmailWithSendMail
      
      //***************************************************************
      //PURPOSE: Sends and email message using the CDONTS components
      //NOTE: The CDONTS components were created by Microsoft to
      //       simplify the process of sending email. These components
      //       are distributed with IIS so if IIS is your webserver
      //       you can probably use these components.
      //***************************************************************
      procedure SendEmailWithCDONTS;
      const
         NORMAL = 1;
         //LOW = 0;
         //HIGH = 2;
      var
         objMail : object;
         iLastError : integer;
         strError : string;
      begin //SendEmailWithCDONTS
         traperrors(false);
         objMail := CreateObject('CDONTS.NewMail');
         traperrors(true);
         iLastError := getlasterror;
         if iLastError <> 0 then
            begin
               str(iLastError:1, strError);
               strError := strError+' ' + errors[1].description;
               DisplayError('#'+strError);
            end;
         objMail.Send(strFrom , strTo, strSubject, strBody, NORMAL);
         dispose(objMail);
      end;  //SendEmailWithCDONTS

   begin //SendEmail
      //If this application is running under a Unix-Like platform then
      // use sendmail to send email message
      //If not use the CDONTS object
      if UnixPlatform then
         SendEmailWithSendMail
      else
         SendEmailWithCDONTS
   end;  //SendEmail

   //********************************************************************
   //PURPOSE: Reads the CGI request information and stores it in a buffer
   //NOTE: The environment variable "REQUEST_METHOD" is used to determine
   //       whether a GET or POST request was made, and calls the
   //       appropriate procedure to read the data.
   //This procedure is generic and can probably be used in almost any
   //CGI application. The only thing likely to change is the size of the
   // buffer used to store the CGI request information.
   //********************************************************************
   procedure GetCGIData(var strData : Buffer);
   var
      RequestMethod : string;

      procedure GetRequest(var strData : Buffer);
      begin // GetRequest
         strData := getenv('QUERY_STRING')
      end; // GetRequest

      procedure PostRequest(var strData : Buffer);
      var
         len, i : 0..maxint;
         err : integer;
         ContentLength : string;
         c : char;
      begin // PostRequest
         strData := '';
         ContentLength := getenv('CONTENT_LENGTH');
         if ContentLength <> '' then
            val(ContentLength, len, err)
         else
            len := 0;
         if err > 0 then
            len := 0;
         if len <= MaxBuffer then
            for i := 1 to len do
               begin
                  read(c);
                  strData := strData + c
               end
      end; // PostRequest

   begin // GetCGIData
      RequestMethod := getenv('REQUEST_METHOD');
      if RequestMethod = 'GET' then
         GetRequest(strData)
      else
         PostRequest(strData);
   end; // GetCGIData

   //********************************************************************
   //PURPOSE: Processes the CGI request information that was earlier
   //    stored in the buffer. This involves seperating the information
   //    in the buffer into name/value pairs and then decoding them.
   //    CGI request information is passed to applications in the following
   //    form:
   //          name=value
   //    called name/value pairs. Each name/value pair is encoded, and then
   //    joined together in one big block of data seperated by '&'.
   //    So to process the name/value pairs the process is reversed.
   //    The individual name/value pairs are seperated from the block and
   //    then decoded, then the "name" part is seperated from the "value"
   //    part, and finally the procedure ProcessNameValuePair is called to
   //    do something with the the name/value pair.
   //Most of this procedure is generic and can be used in almost any
   //    CGI application, the only part of this procedure that is
   //    specific to this application is what goes on inside
   //    "ProcessNameValuePair". In this case some values get assigned to
   //    some global variables.
   //********************************************************************
   procedure ProcessCGIData(var strData : Buffer);
   var
      i, num, p : integer;
      EncodedVariable, DecodedVariable, name, value : Buffer;

      procedure ProcessNameValuePair;
      begin //ProcessNameValuePair
         if name = 'to' then
            strTo := value
         else if name = 'subject' then
            strSubject := value
         else if name = 'from' then
            strFrom := value
         else if name = 'body' then
            strBody := value
         else if name = 'error_link' then
            strErrorLink := value
         else if name = 'success_link' then
            strSuccessLink := value
         else if name = 'success_message' then
            strSuccessMessage := value
      end;  //ProcessNameValuePair

   begin // ProcessCGIData
      num := CountWords(strData, '&');
      for i := 1 to num do
         begin
            EncodedVariable := CopyWord(strData, i, '&');
            DecodedVariable := URLDecode(EncodedVariable);
            p := pos('=', DecodedVariable);
            if p > 0 then
               begin
                  name := lowercase(trim(copy(DecodedVariable, 1, p-1)));
                  value := trim(copy(DecodedVariable, p+1));
                  ProcessNameValuePair;
               end
         end
   end; // ProcessCGIData

   //*******************************************************
   //PURPOSE: Writes the HTTP response header
   //*******************************************************
   procedure WriteResponseHeader;
   begin //WriteResponseHeader
      writeln('Content-type: text/html');
      writeln;
   end;  //WriteResponseHeader

   //**************************************************************
   //PURPOSE: Generates the HTML response and possibly send
   //          the email message.
   //**************************************************************
   procedure GenerateResponse;
   var
      i : integer;
      InvalidTo : boolean;

      procedure GenerateHTMLHeader;
      begin
         writeln('<html>');
         writeln('<head>');
         writeln('<title>CGIMail 1.0</title>');
         writeln('<meta name="GENERATOR" content="CGIMail 1.0">');
         writeln('<meta name="COPYRIGHT" content="Copyright (c) 2001, Stuart King.">');
         writeln('</head>');
      end;

      procedure GenerateHTMLFooter;
      begin
         writeln('<hr>');
         writeln('<p>');
         writeln('CGIMail 1.0 Copyright &copy; 2001, Stuart King<br>');
         writeln('Home page <a href="http://www.irietools.com/">www.irietools.com</a>');
         writeln('</p>');
         writeln('</body>');
         writeln('</html>');
      end;

      //*******************************************************************
      //PURPOSE: Generates the response when the email recipient (in strTo)
      //          is not specified.
      //*******************************************************************
      procedure GenerateNoTo;
      begin
         GenerateHTMLHeader;
         writeln('<body>');
         writeln('<h1>');
         writeln('Email Address (Recipient) Required');
         writeln('</h1>');
         writeln('<p>The address of the email recipient was not specified.</p>');
         if strErrorLink <> '' then
            writeln('<p>Please click <a href="', EscapeCharacters(strErrorLink), '">here</a> to continue.</p>');
         GenerateHTMLFooter;
      end;

      //*******************************************************************
      //PURPOSE: Generates the response when the email recipient (in strTo)
      //          is invalid.
      //*******************************************************************
      procedure GenerateInvalidTo(strTo : string);
      begin
         GenerateHTMLHeader;
         writeln('<body>');
         writeln('<h1>');
         writeln('Invalid Email Address (Recipient)');
         writeln('</h1>');
         writeln('<p>The following email address (', EscapeCharacters(strTo), ') is invalid.</p>');
         if strErrorLink <> '' then
            writeln('<p>Please click <a href="', EscapeCharacters(strErrorLink), '">here</a> to continue.</p>');
         GenerateHTMLFooter;
      end;

      //*******************************************************************
      //PURPOSE: Generates the response when the email sender (in strFrom)
      //          is invalid.
      //*******************************************************************
      procedure GenerateInvalidFrom;
      begin
         GenerateHTMLHeader;
         writeln('<body>');
         writeln('<h1>');
         writeln('Invalid Email Address (Sender)');
         writeln('</h1>');
         writeln('<p>The following email address (', EscapeCharacters(strFrom), ') is invalid.</p>');
         if strErrorLink <> '' then
            writeln('<p>Please click <a href="', EscapeCharacters(strErrorLink), '">here</a> to continue.</p>');
         GenerateHTMLFooter;
      end;

      //*******************************************************************
      //PURPOSE: Generates the response when everything is OK and the email
      //          message has been sent.
      //*******************************************************************
      procedure GenerateEmailSent;
      begin
         GenerateHTMLHeader;
         writeln('<body>');
         writeln(strSuccessMessage);
         if strSuccessLink <> '' then
            writeln('<p>Please click <a href="', EscapeCharacters(strSuccessLink), '">here</a> to continue.</p>');
         GenerateHTMLFooter;
      end;

   begin // GenerateResponse
      if strTo = '' then
         GenerateNoTo
      else
         begin
            InvalidTo := false;
            //Check each recipient address, if an invalid recipient is found
            // generate the invalid recipient reponse and set the InValidTo
            // flag so that no other responses get generated.
            for i := 1 to CountWords(strTo, ',') do
               if (not InvalidTo) and (not IsValidEmailAddress(CopyWord(strTo, i, ','))) then
                  begin
                     GenerateInvalidTo(CopyWord(strTo, i, ','));
                     InvalidTo := true
                  end;
            //If thr InvalidTo flag has not been set then check the
            // sender's email address if it was specified.
            if not InvalidTo then
               if (strFrom <> '') and (not IsValidEmailAddress(strFrom)) then
                  GenerateInvalidFrom
               else
                  begin
                     //Everything is OK so send the email and generate
                     //the email send response.
                     SendEmail(strFrom, strTo, strSubject, strBody);
                     GenerateEmailSent
                  end
         end;
   end; // GenerateResponse

begin
   WriteResponseHeader;

   Initialize;

   ReadConfigFile;

   GetCGIData(strCGIData);

   ProcessCGIData(strCGIData);

   GenerateResponse;
end.
