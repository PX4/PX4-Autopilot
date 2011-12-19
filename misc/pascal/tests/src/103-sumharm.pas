PROGRAM sumharmonics (input, output);
  CONST
    firstterm = 2;
  VAR
    numerator, denominator,
    lastterm, termcount : integer;

  PROCEDURE lowterm (VAR num, den : integer);
    VAR
      numcopy, dencopy, remainder : integer;
    BEGIN
      numcopy := num;
      dencopy := den;
      WHILE dencopy <> 0 DO
        BEGIN
          remainder := numcopy MOD dencopy;
          numcopy := dencopy;
          dencopy := remainder;
        END; { while }
      IF numcopy > 1
        THEN
          BEGIN
            num := num DIV numcopy;
            den := den DIV numcopy;
          END
    END; { lowterm }

  PROCEDURE addrationals (VAR num1, den1 : integer;
                              num2, den2 : integer);
    BEGIN
      num1 := num1 * den2 + num2 * den1;
      den1 := den1 * den2;
    END; { addrationals }

  BEGIN { sumharmonics }
    numerator := 1;
    denominator := 1;
    READ (lastterm);
    FOR termcount := firstterm TO lastterm DO
      BEGIN
        addrationals (numerator, denominator, 1, termcount);
        lowterm (numerator, denominator);
        WRITELN (numerator, '/', denominator)
      END; { for }
  END. {sumharmonics}

