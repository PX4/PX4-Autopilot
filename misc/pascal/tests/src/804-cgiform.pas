(********************************************************************************************
** PROGRAM     : cgiform
** VERSION     : 1.0.0
** DESCRIPTION : Demonstrates how to process HTML forms in CGI programs.
** AUTHOR      : Stuart King
** COPYRIGHT   : Copyright (c) Irie Tools, 2002. All Rights Reserved.
** NOTES       :
**    This sample program is distributed with Irie Pascal, and was written to illustrate
** how to process HTML forms. To make best use of this sample you should have a basic
** understanding of Pascal as well as a basic understanding of the Common Gateway Interface
** (CGI).
**
**    HTML forms provide a way for websites to receive input from visitors, and to process
** this input in some way. HTML forms contain different kinds of input elements in order
** to conveniently receive different kinds of visitor input. The most common kinds of
** input elements are:
**   1. One line text entry boxes
**   2. Multi-line text entry boxes
**   3. Checkboxes
**   4. Radio buttons
**   5. Hidden fields
**   6. Selection lists (Drop-down menus and List boxes).
**   7. Reset button
**   8. Submit button
**
**    HTML forms have action attributes that indicate what should happen when the submit
** button is clicked. It is very common for the action attribute to point at the URL of
** a CGI program. In this case when the submit button is pressed the CGI program is executed
** and the form's input is passed to the program for processing. Each input element in the
** form has a name and the form's input is sent to the CGI program in the form of name/value
** pairs, where the values are the input received by the input elements.
**
**    What this program actually does is retrieve any form input passed to it. If it receives
** form input then this program just displays the name/value pairs, along with a link that
** can be used to execute the program again. If the program does not receive any form input
** then it displays a form with a variaty of input elements. The form's action element points
** back to the program, so that the program will receive the input from the form when the
** submit button is pressed.
**********************************************************************************************)
program cgiform;
const
	MAX_BUFFER = 8000;
	MAX_NAME = 20;
	MAX_VALUE = 400;
type
	positive = 0..maxint;
	BufferType = string[MAX_BUFFER];
	NameValuePair = record
		Name : string[MAX_NAME];
		Value : string[MAX_VALUE]
	end;
var
	buffer : BufferType;
	NameValuePairs : list of NameValuePair;
	ScriptName : filename;

	function EscapeCharacters(s : string) : string; forward;

	//PURPOSE: Initializes the program
	//NOTES:
	//   Initializes the list that will be used to store the name/value pairs
	//passed to the program. The program also retrieves it's name so that it can
	//refer to itself in the generated response.
	procedure Initialize;
	begin (* Initialize *)
		new(NameValuePairs);
		ScriptName := getenv('SCRIPT_NAME');
	end; (* Initialize *)

	//PURPOSE: Retrieves the information passed to the CGI applications.
	//GLOBAL(s) - buffer - Used to store the GET or POST information passed to the CGI program
	procedure GetCGIData;
	var
		RequestMethod : string;

		//PURPOSE: Retrieves information sent to by a GET request (i.e. in the QUERY_STRING
		//         environment variable).
		procedure GetRequest;
		begin (* GetRequest *)
			buffer := getenv('QUERY_STRING')
		end; (* GetRequest *)

		//PURPOSE: Retrieves information sent to by a POST request (i.e. through the standard
		//         input stream, with the length of the data in the environment variable
		//         CONTENT_LENGTH).
		procedure PostRequest;
		var
			len, i : positive;
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
			if len <= MAX_BUFFER then
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

	//PURPOSE: Process the data passed to the program.
	//NOTES: This is the main part of the program. After retreiving
	//       the information passed to the program this procedure is
	//       called to perform the required processing.
	procedure ProcessCGIData;
	var
		i, num, p : integer;
		EncodedVariable, DecodedVariable, name, value : string;

		//PURPOSE: Processes the named value pairs sent with the GET or POST request.
		//         Which in this case is the information entered by the user about the
		//         cookie to add or delete.
		//ARGUMENT(s): name - name part of the name/value pair
		//             value - value part of name/value pair
		//NOTES:
		//    The information entered by the user is sent as name/value pairs (i.e. name-value)
		//with the name part being the name of the form element holding the information and
		//the value part being the actual information held by the form element.
		procedure ProcessNameValuePair(var name, value : string);
		var
			pair : NameValuePair;
		begin (* ProcessNameValuePair *)
			pair.name := name;
			pair.value := value;
			insert(pair, NameValuePairs);
		end; (* ProcessNameValuePair *)

	begin (* ProcessCGIData *)
		//Retrieve each name/value pair from the form and processes them.
		num := CountWords(buffer, '&');
		for i := 1 to num do
			begin
				EncodedVariable := CopyWord(buffer, i, '&');
				DecodedVariable := URLDecode(EncodedVariable);
				p := pos('=', DecodedVariable);
				if p > 0 then
					begin
						name := lowercase(trim(copy(DecodedVariable, 1, p-1)));
						value := trim(copy(DecodedVariable, p+1));
						ProcessNameValuePair(name, value);
					end
			end;
	end; (* ProcessCGIData *)

	//PURPOSE: Generates the response to send back to the browser.
	procedure GenerateResponse;

		procedure GenerateHeader;
		begin (* GenerateHeader *)
			//Generate the response headers (including the blank line at the end).
			writeln('content-type: text/html');
			writeln;

			writeln('<html>');
			writeln('<head>');
			writeln('<title>Irie Pascal sample CGI application</title>');
			writeln('<h1>CGIFORM</h1>');
			writeln('<h2>This program displays the data entered into a form.</h2>');
			writeln('</head>');
			writeln('  <hr>');
		end; (* GenerateHeader *)

		procedure GenerateBody;

			procedure WriteForm;
			begin (* WriteForm *)
				writeln('<form method="POST" action="', ScriptName, '">');
				writeln('  <h2>One Line Text Box:</h2>');
				writeln('  <p>OneLine <input type="text" name="OneLine" size="20"></p>');
				writeln('  <hr>');
				writeln('  <h2>Scrolling Text Box:</h2>');
				writeln('  <p>Scrolling <textarea rows="2" name="Scrolling" cols="20"></textarea></p>');
				writeln('  <hr>');
				writeln('  <h2>Check Boxes</h2>');
				writeln('  <p>Box1 <input type="checkbox" name="Box1" value="1">');
				writeln('  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Box2 <input type="checkbox" name="Box2"');
				writeln('  value="1"></p>');
				writeln('  <p>Box3 <input type="checkbox" name="Box3" value="1">');
				writeln('  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Box4 <input type="checkbox" name="Box4"');
				writeln('  value="1"></p>');
				writeln('  <hr>');
				writeln('  <h2>Radio Buttons</h2>');
				writeln('  <p>Radio1 <input type="radio" value="1" checked name="Radio1">');
				writeln('  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Radio2 <input type="radio" name="Radio2"');
				writeln('  value="2"></p>');
				writeln('  <p>Radio3 <input type="radio" name="Radio3" value="3">');
				writeln('  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Radio4 <input type="radio" name="Radio4"');
				writeln('  value="4"></p>');
				writeln('  <hr>');
				writeln('  <h2>Drop-Down Menu</h2>');
				writeln('  <p>DropDown <select name="DropDown" size="1">');
				writeln('    <option value="Choice1">Choice1</option>');
				writeln('    <option value="Choice2">Choice2</option>');
				writeln('    <option value="Choice3">Choice3</option>');
				writeln('    <option value="Choice4">Choice4</option>');
				writeln('    <option value="Choice5">Choice5</option>');
				writeln('    <option value="Choice6">Choice6</option>');
				writeln('  </select></p>');
				writeln('  <hr>');
				writeln('  <p><input type="submit" value="Submit" name="Submit"><input type="reset" value="Reset"');
				writeln('  name="Reset"></p>');
				writeln('</form>');
			end; (* WriteForm *)

			procedure WriteFormData;
			var
				pair : NameValuePair;
				i : positive;
			begin (* WriteFormData *)
				writeln('<h1>Form Data</h1>');
				for i := 1 to length(NameValuePairs) do
					begin
						pair := NameValuePairs[i];
						writeln('<h3>', EscapeCharacters(pair.name), ' = ', EscapeCharacters(pair.value), '</h3>');
					end;
				writeln('<hr>');
				writeln('<p>Click <a href="', ScriptName, '">here</a> to go back to the form.</p>');
			end; (* WriteFormData *)

		begin (* GenerateBody *)
			writeln('<body>');

 			if length(NameValuePairs) = 0 then
 				begin
					//Generate the HTML for the form
					WriteForm;
				end
			else
				begin
					//Generate the HTML that displays the form data
					WriteFormData;
				end;

			writeln('</body>');
		end; (* GenerateBody *)

		procedure GenerateFooter;
		begin (* GenerateFooter *)
			writeln('</html>');
		end; (* GenerateFooter *)

	begin (* GenerateResponse *)
		GenerateHeader;
		GenerateBody;
		GenerateFooter;
	end; (* GenerateResponse *)
              
    procedure Shutdown;
	begin (* Shutdown *)
		dispose(NameValuePairs);
	end; (* Shutdown *)

	//*************************************************************************
	//PURPOSE: This function converts certain characters that have a
	//         special meaning in HTML documents to their HTML representation.
	//ARGUMENT(s): s - The string to be escaped.
	//RETURNS: The string with all special characters escaped.
	//NOTES: The characters converted are < > "
	function EscapeCharacters;
	const
		LessThanChar = '<';
		GreaterThanChar = '>';
		QuoteChar = '"';
		HTMLLessThan = '&lt;';
		HTMLGreaterThan = '&gt;';
		HTMLQuote = '&quot;';
	var
		i : positive;

		procedure ReplaceChar(var strBuffer : string; strReplace : string; i : positive);
		begin (* ReplaceChar *)
			delete(strBuffer, i, 1);
			insert(strReplace, strBuffer, i)
		end; (* ReplaceChar *)

	begin (* EscapeCharacters *)
		repeat
			i := pos(LessThanChar, s, 1);
			if i > 0 then
				ReplaceChar(s, HTMLLessThan, i)
		until i = 0;

		repeat
			i := pos(GreaterThanChar, s, 1);
			if i > 0 then
				ReplaceChar(s, HTMLGreaterThan, i)
		until i = 0;

		repeat
			i := pos(QuoteChar, s, 1);
			if i > 0 then
				ReplaceChar(s, HTMLQuote, i)
		until i = 0;

		EscapeCharacters := s;
	end; (* EscapeCharacters *)

begin
	Initialize;

	GetCGIData;

	ProcessCGIData;

	GenerateResponse;

	Shutdown;
end.
