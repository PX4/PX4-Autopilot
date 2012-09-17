(********************************************************************************************
** PROGRAM     : cgicook
** VERSION     : 1.0.0
** DESCRIPTION : Demonstrates how to use cookies in CGI programs.
** AUTHOR      : Stuart King
** COPYRIGHT   : Copyright (c) Irie Tools, 2002. All Rights Reserved.
** NOTES       :
**    This sample program is distributed with Irie Pascal, and was written to illustrate
** how to use cookies (i.e. how to read, write, and delete cookies). To make best use of
** this sample you should have a basic understanding of Pascal as well as a basic
** understanding of the Common Gateway Interface (CGI).
**
**    Before describing how to use cookies, here is a very brief description of what
** cookies are and what they are used for. Cookies are named pieces of information that a
** website can ask a client (usually a browser) to store on its behalf. This information
** can be sent back to the website (and any other website in the cookies domain) whenever
** the browser sends a request to the website. Cookies are usually stored on the client's
** hard drive and can persist for weeks, months, or even years. This makes cookies very
** useful for 'remembering' information about website visitors.
** IMPORTANT: The cookies sent between the client and the website are sent in text format
** and are visible to any snooper, so sensitive information should either be encrypted
** or sent only over a secure connection.
**
**    Cookies are written by sending a "Set-Cookie" response header back to the client. The
** syntax for the "Set-Cookie" header is given below:
**
** Set-Cookie: name=value; domain=.domain.com; path=/path;
**            expires=Day, dd-Mon-yyyy hh:mm:ss GMT; secure
** Where
**   "name" is the name of the cookie.
**   "value" is the information stored by the cookie and must be specified. If you want
**           to delete a cookie then specify an expiry date that has already passed.
**   "domain" restricts the domains that will receive the cookie. The client should only send
**            the cookie to matching domains. Domains are matched from right to left. So
**            "domain=.irietools.com" matches "www.irietools.com" and "info.irietools.com".
**            If "domain" is specified it must match the domain of the website setting the
**            cookie. "domain" can't specify a top-level domain like ".com" or ".edu.jm".
**            If "domain" is not specified then the domain name of the website setting
**            the cookie is used.
**   "path" restricts the URLs within a domain that will receive the cookie. Paths are
**          matched from left to right and trailing /'s are ignored. If "path" is not
**          specified then the full path of the request is used.
**   "expires" specifies when the cookie should expire. The format of the expiry info
**          must be exactly as shown above:
**          "Day" is the three letter day of the week (Mon, Tue, Wed, Thu, Fri, Sat, or Sun).
**          "dd" is the two digit day of the month (01-31).
**          "Mon" is the  three letter abbreviated month name
**             (Jan, Feb, Mar, Apr, Jun, Jul, Aug, Sep, Oct, Nov, Dec).
**          For example: "Mon, 16-Sep-2002 17:03:48 GMT".
**          If "expire" is not specified then the cookie is stored memory until the client
**          exits.
**   "secure" indicates that the cookie should only be sent over a secure connection.
**
**   The browser sends cookies back to the website in the form of Cookie headers. The
** format for Cookie headers is given below:
**
** Cookie: Cookie1; ...; CookieN
** Where
**    each Cookiue is given as: name=value
**
**    Only the "name" and the "value" of the cookies is returned, the other information
** such as the "domain" is not returned. CGI programs can't read the Cookie headers
** directly, so the webserver will make the Cookie header information available to the
** CGI program in the HTTP_COOKIE environment variable.
**********************************************************************************************)
program cookies;
const
	MAX_BUFFER = 4096;
	MAX_COOKIE_DATA = 200;
	//SCRIPT_NAME = '/irietools/cgibin/cookies.exe';
type
	positive = 0..maxint;
	BufferType = string[MAX_BUFFER];
	CookieDataType = string[MAX_COOKIE_DATA];
	DayOfWeek = 1..7;
	Days = 1..31;
	Months = 1..12;
	date = record
		day : Days;
		month : Months;
		year : integer;
		dow : DayOfWeek
	end;
	Cookie = record
		name : string;
		value : CookieDataType
	end;
	CookieList = list of Cookie;
var
	buffer : BufferType;
	strSetCookie : BufferType;
	DaysInMonth : array[Months] of Days;
	DaysElapsed : array[Months] of integer;
	DayOfWeekShortNames : array[DayOfWeek] of string[3];
	MonthShortNames : array[Months] of string[3];
	Cookies : CookieList;

	procedure DateToInt(dt : date; var iDays : integer); forward;
	procedure IntToDate(iDays : integer; var dt : date); forward;
	procedure CalculateDayOfWeek(var dt : date); forward;
	function urlencode(strIn : string) : string; forward;
	function EscapeCharacters(s : string) : string; forward;

	//PURPOSE: Performs program initialization
	procedure Initialize;
	var
		m : months;
	begin (* Initialize *)
		DaysInMonth[1] := 31;
		DaysInMonth[2] := 28;
		DaysInMonth[3] := 31;
		DaysInMonth[4] := 30;
		DaysInMonth[5] := 31;
		DaysInMonth[6] := 30;
		DaysInMonth[7] := 31;
		DaysInMonth[8] := 31;
		DaysInMonth[9] := 30;
		DaysInMonth[10] := 31;
		DaysInMonth[11] := 30;
		DaysInMonth[12] := 31;

		DaysElapsed[1] := 0;
		for m := 2 to 12 do
			DaysElapsed[m] := DaysElapsed[m-1]+DaysInMonth[m-1];

		DayOfWeekShortNames[1] := 'Sun';
		DayOfWeekShortNames[2] := 'Mon';
		DayOfWeekShortNames[3] := 'Tue';
		DayOfWeekShortNames[4] := 'Wed';
		DayOfWeekShortNames[5] := 'Thu';
		DayOfWeekShortNames[6] := 'Fri';
		DayOfWeekShortNames[7] := 'Sat';

		MonthShortNames[1] := 'Jan';
		MonthShortNames[2] := 'Feb';
		MonthShortNames[3] := 'Mar';
		MonthShortNames[4] := 'Apr';
		MonthShortNames[5] := 'May';
		MonthShortNames[6] := 'Jun';
		MonthShortNames[7] := 'Jul';
		MonthShortNames[8] := 'Aug';
		MonthShortNames[9] := 'Sep';
		MonthShortNames[10] := 'Oct';
		MonthShortNames[11] := 'Nov';
		MonthShortNames[12] := 'Dec';

		new(Cookies);	//initialize the list of cookies
	end; (* Initialize *)

	//PURPOSE: Retrieves the information passed to the CGI applications.
	//PARAMETER(s): cl - Used to store all the cookies that were passed to the CGI program
	//GLOBAL(s) - buffer - Used to store the GET or POST information passed to the CGI program
	procedure GetCGIData(var cl : CookieList);
	var
		RequestMethod : string;

		//PURPOSE: Retrieves the cookies passed to the CGI program and puts then in the list.
		//PARAMETER(s): cl - Used to store all the cookies that were passed to the CGI program
		procedure GetCookieInfo(var cl : CookieList);
		var
			data : BufferType;
			iNumCookies, iCurrCookie : positive;
			iPos, iLen : positive;
			strCurrCookie : CookieDataType;
			c : Cookie;
		begin (* GetCookieInfo *)
			//The cookies are stored in the environment variable HTTP_COOKIE as space
			// deliminated words with trailing semi-colons after each word (except the last word).
			//Each word being a single cookie.
			//Retrieve the cookies
			data := getenv('HTTP_COOKIE');

			//Retrieve the number words/cookies.
			iNumCookies := countwords(data);

			//Retrieve each cookie and use the "=" to seperate the name from the value
			//then insert the cookie into the list.
			for iCurrCookie := 1 to iNumCookies do
				begin
					strCurrCookie := copyword(data, iCurrCookie);
					iPos := pos('=', strCurrCookie);
					if iPos <> 0 then
						begin
							c.name := trim(copy(strCurrCookie, 1, iPos-1));
							c.value := trim(copy(strCurrCookie, iPos+1));
							iLen := length(c.value);
							if (iLen<>0) and (c.value[iLen]=';') then
								c.value := copy(c.value, 1, iLen-1);
							insert(c, cl);
						end;
				end;
		end; (* GetCookieInfo *)

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
		GetCookieInfo(cl);
		RequestMethod := getenv('REQUEST_METHOD');
		if RequestMethod = 'GET' then
			GetRequest
		else
			PostRequest
	end; (* GetCGIData *)

	//PURPOSE: Process the data passed to the program.
	//NOTES: This is the main part of the program. After retreiving
	//       the information passed to the program this procedure is
	//       called to perform the required processing. This program receives to kinds
	//       of information:
	//       1. Cookies sent back by the brower. These have already been retrieved by the
	//            the procedure "GetCGIData" and do not require further processing here.
	//            The procedure 'GenerateResponse' will display the retrieved cookies.
	//       2. Information entered by the user about a cookie to add or delete. This
	//           information needs to formated and put in the global 'strSetCookie', so that
	//           it can be included in the response by the procedure 'GenerateResponse'.
	procedure ProcessCGIData;
	var
		i, num, p : integer;
		EncodedVariable, DecodedVariable, name, value : string;
		strName : string;
		strValue : string;
		strExpiry : string;
		blnDelete : boolean;

		//PURPOSE: Converts a expiry date into a string in the format required
		//         by the Set-Cookie header (i.e. Day, dd-Mon-yyyy hh:mm:ss GMT
		//ARGUMENT(s): dt - Expire date to convert.
		//RETURNS: The expiry date/time in Set-Cookie format
		//NOTES:
		//    This function just hard-codes the last second in the day "23:59:59"
		// instead of using the current time or allowing the expiry time to be specified.
		//This is done because
		// 1. It is easier and this is just a sample program
		// 2. Usually cookie will be set to expire either when the browser closes
		//    or after many days have passed, so the exact time is not important.
		function FormatCookieExpiryString(dt : date) : string;
		var
			s : string;

			//PURPOSE: Converts an integer into a zero-padded string
			//ARGUMENT(s):
			//   1. i - The integer to convert
			//   2. len - The length of the string to return
			//RETURNS:
			//   The zero padded string
			function Int2String(i : integer; len : integer) : string;
			var
				S : string;
			begin (* Int2String *)
				str(i:1, s);
				while length(s) < len do
					s := '0' + s;
				Int2String := s
			end; (* Int2String *)

		begin (* FormatCookieExpiryString *)
			CalculateDayOfWeek(dt);
			s := DayOfWeekShortNames[dt.dow]+', ';
			s := s + Int2String(dt.day, 2)+'-'+MonthShortNames[dt.month]+'-'+Int2String(dt.year, 4)+' ';
			s := s + '23:59:59 GMT';
			FormatCookieExpiryString := s;
		end; (* FormatCookieExpiryString *)

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
			dt : date;
			dow : integer;
			iDays, iErr : integer;
			iDate : integer;
		begin
			//If the name is 'txtname' then this is the name of the cookie.
			//The name is urlencoded in case it contains special characters
			if name='txtname' then
				strName := urlencode(value)
			//If the name is 'txtvalue' then this is the value of the cookie.
			//The value is urlencoded in case it contains special characters
			else if name='txtvalue' then
				strValue := urlencode(value)
			//If the name is 'txtdays' then this is the number of days before the cookie
			//expires. The value is converted to an integer and if the conversion is
			//successful the value is added to the current date to get the expiry date
			//and this date is formatted for use in a Set-Cookie header.
			else if name='txtdays' then
				begin
					val(value, iDays, iErr);
					if iErr=0 then
						begin
							getdate(dt.year, dt.month, dt.day, dow);//Get current date
							DateToInt(dt, iDate);	//Convert current date to number of days since Jan 1, 0001 AD
							iDate := iDate + iDays; //Add in the number of days to expire
							IntToDate(iDate, dt);	//Convert the result back into a date
							strExpiry := FormatCookieExpiryString(dt); //Format date
						end;
				end
			//If the name is 'cmddelete' then the cookie should be deleted not added.
			//The cookie is deleted by setting an expiry date that has already passed.
			else if name='cmddelete' then
				begin
					getdate(dt.year, dt.month, dt.day, dow);
					DateToInt(dt, iDate);
					iDate := iDate - 2;
					IntToDate(iDate, dt);
					strExpiry := FormatCookieExpiryString(dt);
					blnDelete := true
				end
			else
				;
		end;

	begin (* ProcessCGIData *)
		strSetCookie := '';
		strName := '';
		strValue := '';
		strExpiry := '';
		blnDelete := false;

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

		//If after processing each name/value pair we have enough data for a
		//"Set-Cookie" header then generate a "Set-Cookie" header in the global
		//"strSetCookie".
		//if (strName<>'') and ((strValue<>'') or blnDelete) and (strExpiry<>'') then
		if (strName<>'') and ((strValue='') or (strExpiry<>'')) then
			begin
				//Set-Cookie: name=value; domain=.irietools.com; path=/cgibin;
				//            expires=Tue, 03-Sep-2002 20:42:19 GMT; secure
				strSetCookie := 'Set-Cookie: ' + strName + '=' + strValue + ';';
				if strExpiry<>'' then
					strSetCookie := strSetCookie + ' expires=' + strExpiry;
			end;
	end; (* ProcessCGIData *)

	//PURPOSE: Generates the response to send back to the browser.
	//NOTES:
	//   Most of the HTML generated by this procedure actually comes from a template file.
	//The HTML from the template file is common to almost all the pages on this website.
	//The HTML specific to this program (i.e. the form that accepts the cookie information
	//from the user), is generated by the local procedure 'GenerateBodyData'. So this procedure
	//basically reads the template file and sends most of it to the webserver except for
	//a small section which it repalces with the output of the procedure 'GenerateBodyData'.
	//The template file is divided into sections using HTML comments. The start of a section
	//is marked by the following
	//
	// <!--section SectionName-->
	//
	//Where "SectionName" is the name of the section.
	//
	//The end of a section is marked by
	//
	// <!--/section-->
	//
	//What this procedure actually does is read the template file and write the contents
	//of all the sections except one to the webserver. The section not written to the webserver
	//is called "bodydata". Instead of writing the contents of the section "bodydata" to the
	//webserver the ouput of the procedure "GenerateBodyData" is sent to the webserver.
	//This is done because it is much easier to work with HTML using an HTML editor rather
	//than using embedded writeln's inside a CGI program. So an HTML editor is used to
	//create and update the template file and the portion of the generated output that
	//is specific to this program is generated by "GenerateBodyData". You will notice
	//when you run this program that the page generated looks alot like the other pages
	//on the website because most of it comes from the template file.
	procedure GenerateResponse;
	const
		TEMPLATE_FILE = 'template.html';
		START_SECTION = '<!--section';
		END_SECTION = '<!--/section-->';
		SECTION_NAME = 'bodydata';
	var
		f : text;
		line : BufferType;
		blnSkipping : boolean;

		//PURPOSE: Generates the portion of the response page that is specific to this
		//         program (the rest of the response page comes from the template file).
		//NOTES:
		//    Tables are used to organise the layout of the generated pages on this website.
		//The portion of the response page generated by this procedure is a single row of
		//a table.  Unless you are an HTML expert the easiest way to understand the HTML
		//generated by this procedure is probably to run this program and view the
		//response pages generated.
		procedure GenerateBodyData;
		var
			iNumCookies, iCurrCookie : positive;
			c : Cookie;
		begin (* GenerateBodyData *)
			writeln('<tr>');
            writeln('<td width="100%">');
 
 			//Generate the HTML to display the cookies returned by the browser.
			writeln('<h1>Current Cookies</h1>');
			iNumCookies := length(Cookies);	//Number of cookies returned by the browser
			if iNumCookies=0 then
				writeln('<p>None</p>')
			else
				//For each cookie returned write it's name and value
				//NOTE: The name and value are escaped in case they contain
				//special characters.
				for iCurrCookie := 1 to iNumCookies do
					begin
						c := Cookies[iCurrCookie];
						writeln('<p>', EscapeCharacters(urldecode(c.name)), '=', EscapeCharacters(urldecode(c.value)),'</p>');
					end;
			writeln('<hr>');
			writeln('<p>', getenv('SCRIPT_NAME'), '</p>');
			writeln('<hr>');

			//Generate the HTML for the form
			writeln('<form method="POST" action="', getenv('SCRIPT_NAME'), '">');
			writeln('<p><big><strong>Name:</strong></big> <input type="text" name="txtName" size="25"></p>');
			writeln('<p><big><strong>Value:</strong></big> <input type="text" name="txtValue" size="56"></p>');
			writeln('<p><big><strong>Expiries in:</strong></big> <input type="text" name="txtDays" size="5"> <big><strong>Days</strong></big></p>');
			writeln('<p><input type="submit" value=" Add Cookie " name="cmdAdd">');
			writeln('<input type="submit" value=" Delete Cookie " name="cmdDelete">');
			writeln('<input type="reset" value=" Reset " name="cmdReset"></p>');
			writeln('</form>');

			writeln('</td>');
			writeln('</tr>');
		end; (* GenerateBodyData *)

	begin (* GenerateResponse *)
		//Generate the response headers (including the blank line at the end).
		writeln('content-type: text/html');
		if strSetCookie <> '' then
			writeln(strSetCookie);
		writeln;

		blnSkipping := false;
		reset(f, TEMPLATE_FILE);

		//Read each line in the template file and search for section markers
		//If a target section marker is found then call 'GenerateBodyData' to
		//generate the HTML for that section and set 'blnSkipping' to true so
		//that the contents of that section, from the template file, will be skipped.
		while not eof(f) do
			begin
				readln(f, line);
				line := trim(line);
				if pos(START_SECTION, line) > 0 then
					begin
						if pos(SECTION_NAME, line) > 0 then
						begin
							GenerateBodyData;
							blnSkipping := true;
						end
					end
				else if line=END_SECTION then
					blnSkipping := false
				else if not blnSkipping then
					writeln(line);
			end;
		close(f);
	end; (* GenerateResponse *)

	procedure Shutdown;
	begin (* Shutdown *)
		dispose(Cookies);
	end; (* Shutdown *)

	//PUPOSE: Determines if a year was a leap year.
	function IsLeapYear(year : integer) : boolean;
	begin (* IsLeapYear *)
		IsLeapYear :=
			((year mod 4)=0)
				and
			(
				((year mod 100)<>0)
					or
				((year mod 400)=0)
			);
	end; (* IsLeapYear *)

	//PURPOSE: Converts a date into an integer. The integer represents the
	//         number of days since 'Jan 1, 0001 AD'. Negative values present
	//         the number of days before 'Jan 1, 0001 AD' (i.e. BC dates).
	//ARGUMENT(s): dt - The date to convert to an integer
	//             iDays - The converted date as an integer.
	procedure DateToInt;
	var
		i, year : integer;

		//PURPOSE: Used to help adjust for the leap years when calulating the number
		//         of days that have already elapsed since the year began.
		//RETURNS: +1 if the date is a leap year and the month of february
		//            has already passed.
		// or
		//          0 otherwise
		function LeapYearAdjustment(dt : date) : integer;
		var
			iAdjustment : integer;

		begin (* LeapYearAdjustment *)
			iAdjustment := 0;

			if (IsLeapYear(dt.year)) and (dt.month >= 3) then
				iAdjustment := 1;

			LeapYearAdjustment := iAdjustment;
		end; (* LeapYearAdjustment *)

	begin (* DateToInt *)
		if dt.year < 0 then
			year := dt.year+1	//Adjust for the fact that there was no year 0.
		else
			year := dt.year;
		//Calculate days since Jan 1, 0001 AD ignoring leap years.
		i := (year-1)*365 + DaysElapsed[dt.month] + (dt.day-1);

		//Adjust for leap years except the final year
		i := i + ((year-1) div 4) - ((year-1) div 100) + ((year-1) div 400);

		//Add 1 if the final year is a leap year and february has passed.
		iDays := i + LeapYearAdjustment(dt);
	end; (* DateToInt *)

	//PURPOSE: Coverts an integer, representing the number of days since Jan 1, 0001 AD
	//         to a date.
	//ARGUMENT(s): iDays - The integer to convert to a date.
	//             dt - The converted date.
	//NOTE: There are 365.2425 days in the year adjusting for leap years.
	procedure IntToDate;
	var
		m : Months;
		d : integer;
		iError, iTemp : integer;
	begin
		//First calculate an approximation of the correct answer
		if iDays >= 0 then
			begin
				dt.year := trunc(idays / 365.2425)+1;
				iTemp := iDays - trunc((dt.year-1)*365.2425);
			end
		else
			begin
				dt.year := -(trunc(abs(idays) / 365.2425)+1);
				iTemp := abs(abs(iDays) - trunc((abs(dt.year)-1)*365.2425));
			end;

		dt.month := 12;
		for m := 1 to 12 do
			if iTemp >= DaysElapsed[m] then
				dt.month := m;

		iTemp := iTemp - DaysElapsed[dt.month];

		d := iTemp+1;
		if d > DaysInMonth[dt.month] then
			dt.day := DaysInMonth[dt.month]
		else
			dt.day := d;

		//Now to keep adjusting our approximation until it is exact
		DateToInt(dt, iTemp);
		while iTemp<>iDays do
			begin
				//Number of days that need to be added to the approximation to make it correct.
				iError := iDays-iTemp;

				//If adding the days would make the day of the month negative then
				// then move the approximation to the beginning of the previous month.
				// Adjusting the year if necessary (i.e. it is the first month of the year).
				if (dt.day + iError) < 1 then
					begin
						if dt.month=1 then
							begin
								dt.month := 12;
								dt.year := dt.year - 1;
								if dt.year = 0 then
									dt.year := -1;
							end
						else
							dt.month := dt.month - 1;
						dt.day := 1;
					end
				//If adding the days might take the date into the next month then
				// check if adding the days actually takes the date to Feb 29 on a leap year
				//   if so then adjust the date to Feb 29
				//otherwise move the date to the beginning of the next month.
				else if (dt.day + iError) > DaysInMonth[dt.month] then
					begin
						if (dt.month=2) and (IsLeapYear(dt.year)) and (dt.day+iError=DaysInMonth[dt.month]+1) then
							dt.day := DaysInMonth[dt.month]+1
						else
							begin
								if dt.month=12 then
									begin
										dt.month := 1;
										dt.year := dt.year + 1;
										if dt.year = 0 then
											dt.year := 1;
									end
								else
									dt.month := dt.month + 1;
								dt.day := 1
							end
					end
				//else adding the days results in a valid day of the month then
				// just add the days.
				else
					dt.day := dt.day + iError;

				DateToInt(dt, iTemp);
			end;
	end;

	//PURPOSE: Calculates the day of the week of a given date
	//ARGUMENT(s): dt - The date to calculate the day of the week of.
	//NOTES:
	//   First the date is converted into the number of days since (Jan 1, 0001 AD)
	//Next this value is adjusted with the day of the week of Jan 1, 0001 AD.
	//Next mod 7 is used to give a value in the range 0..6 that repeats every
	//seven days (like the days of the week).
	//Finally the value is incremented by 1 to give a value in the range 1..7
	//
	//The most interesting part of this algorithm is probably the adjustment for the
	//first day of the week. Suppose this adjustment was not made then of the date
	//was Jan 1, 0001 AD then 'DateToInt' would calculate the number of days as 0.
	//Then (0 mod 7) + 1 would give the value 1 or Sunday. But Jan 1, 0001 AD was a
	//Monday, so we need to add a 1 to adjust for this fact. How do I know that
	//Jan 1, 0001 AD was a Monday? I don't, but I do know that day of the week of
	//today. So what I did was calculate the day of the week for today without the
	//adjustment and then added an adjustment that resulted in the correct value.
	procedure CalculateDayOfWeek;
	const
		ADJUST_FOR_FIRST_DAY = 1;	//Apparently Jan 1, 0001 AD was a Monday
	var
		iDays : integer;
	begin (* CalculateDayOfWeek *)
		DateToInt(dt, iDays);
		dt.dow := ((iDays+ADJUST_FOR_FIRST_DAY) mod 7) + 1;
	end; (* CalculateDayOfWeek *)

	//PURPOSE: Returns the URL encoded version of a string.
	//ARGUMENT(s): strIn - The string to be encoded.
	//NOTES:
	//   Spaces are converted to '-' and non-alphanumeric characters are converted
	//to %NN, where NN is the hexidecimal value of the chacters ordinal value. This
	//is the reverse of the built-in function 'urldecode'.
	function urlencode;
	var
		strOut : string;
		i : positive;
		c : char;
		sTemp : string[3];
	begin (* urlencode *)
		strOut := '';
		for i := 1 to length(strIn) do
			begin
				c := strIn[i];
				if c = ' ' then
					strOut := strOut + '+'
				else if isalphanum(c) then
					strOut := strOut + c
				else
					begin
						sTemp := hex(ord(c));
						if length(sTemp)=1 then
							sTemp := '0' + sTemp;
						strOut := strOut + '%' + sTemp
					end;
			end;
		urlencode := strOut;
	end; (* urlencode *)

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
		begin
			delete(strBuffer, i, 1);
			insert(strReplace, strBuffer, i)
		end;

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

	GetCGIData(Cookies);

	ProcessCGIData;

	GenerateResponse;

	Shutdown;
end.
