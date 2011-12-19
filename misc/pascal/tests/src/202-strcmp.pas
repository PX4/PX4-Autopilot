PROGRAM stringops;
CONST
   lexbig     = 'zzzLexically Great';
   lexmiddle1 = 'ZZZLexically Middle+';
   lexmiddle  = 'ZZZLexically Middle';
   lexmiddle2 = 'ZZZLexically Middl';
   lexsmall   = 'AAALexically Small';
   lexnothing = ''
VAR 
   string1, string2 : string;
BEGIN
   IF (lexbig <= lexmiddle) THEN
      writeln('ERROR: ', lexbig, ' <= ', lexmiddle)
   else
      writeln('OKAY:  ', lexbig, ' > ', lexmiddle);

   IF (lexmiddle > lexmiddle1) THEN
      writeln('ERROR: ', lexmiddle, ' > ', lexmiddle1)
   else
      writeln('OKAY:  ', lexmiddle, ' <= ', lexmiddle1);
      
   IF (lexmiddle <> lexmiddle) THEN
      writeln('ERROR: ', lexmiddle, ' <> ', lexmiddle)
   else
      writeln('OKAY:  ', lexmiddle, ' = ', lexmiddle);
      
   IF (lexmiddle = lexnothing) THEN
      writeln('ERROR: ', lexmiddle, ' = ', lexnothing)
   else
      writeln('OKAY:  ', lexmiddle, ' <> ', lexnothing);
      
   IF (lexnothing <> lexnothing) THEN
      writeln('ERROR: ', lexnothing, ' <> ', lexnothing)
   else
      writeln('OKAY:  ', lexnothing, ' = ', lexnothing);
      
   IF (lexmiddle < lexmiddle2) THEN
      writeln('ERROR: ', lexmiddle, ' < ', lexmiddle2)
   else
      writeln('OKAY:  ', lexmiddle, ' >= ', lexmiddle2);

   IF (lexsmall >= lexmiddle) THEN
      writeln('ERROR: ', lexsmall, ' >= ', lexmiddle)
   else
      writeln('OKAY:  ', lexsmall, ' < ', lexmiddle)
      
END.
