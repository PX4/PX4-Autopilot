{ generate the primes between 3..10000 using a
  sieve containing odd integers in this range. }

program primes(output);

const
  wdlength = 59; {implementation dependent}
  maxbit = 58;
  w = 84; {w=n div wdlength div 2}

var
  sieve, primes : array[0..w] of set of 0..maxbit;
  next : record word,bit : integer
         end;
  j,k,t,c : integer; empty:boolean;

begin {initialize}
  for t:=0 to w do
    begin sieve[t] := [0..maxbit]; primes[t] := [] end;
  sieve[0] := sieve[0]-[0]; next.word := 0;
  next.bit := 1; empty := false;

  with next do
  repeat {find next prime}
    while not(bit in sieve[word]) do bit := succ(bit);
    primes[word] := primes[word] + [bit];
    c := 2*bit + 1;
    j := bit; k := word;
    while k<=w do {eliminate}
    begin sieve[k] := sieve[k] - [j];
      k := k + word*2; j := j + c;
      while j>maxbit do
        begin k := k+1; j := j - wdlength
        end
    end;
    if sieve[word]=[] then
      begin empty := true; bit := 0
      end;
    while empty and (word<w) do
      begin word := word+1; empty := sieve[word]=[]
      end
  until empty; {ends with}
end.
