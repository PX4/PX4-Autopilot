// Définition des différentes fonctions

function [List,n]=Load(pathname),
    //List = fscanfMat(pathname,"%5.2f");
    List = fscanfMat(pathname).';
//    List = List.';
    n = length(List)/3;
endfunction

function [List,n] = DeleteZeros(List,n),
    TmpList = [];
    for i = 1:n
        if isequal(List(:,i) ,[0;0;0]) then
        else TmpList = [TmpList,List(:,i)];
        end
    end
    List = TmpList;
    n = length(List)/3;
endfunction

function [List,n] = ApplyMatrix(List,n,Matrix),
    TmpList = List;
    List = [];
        for i = 1:n
        A = TmpList(:,i);
        L = Matrix * A;
        List = [List,L];
    end
endfunction

function [List,n] = ApplyOffset(List,n,Offset),
    TmpList = List;
    List = [];
    for i = 1:3
        List = [List ;TmpList(i,:) - Offset(i)];
    end
endfunction

function [List,n,Offset] = CenterMinMax(List,n,Offset),
    for i = 1:3
        Min(i) = min(List(i,:));
        Max(i) = max(List(i,:));
        Off(i) = (Max(i)+Min(i))/2;
        Offset(i) = Off(i) + Offset(i);
    end
    
    [List,n] = ApplyOffset(List,n,Off);
endfunction

function [List,Offset] = CenterFit(List,Offset),
    function e=G(p,z),
        V = [z(1),z(2),z(3)];
        Off = [p(1),p(2), p(3)];
        e=norm((V+Off).')
    endfunction
    guess = [0;0;0];
    Sol=datafit(G,List,guess);
    
    TmpList = List;
    List = [];
    for i = 1:3
        List = [List ;TmpList(i,:) - Sol(i)];
    end
    Offset = [Sol(1) + Offset(1), Sol(2) + Offset(2), Sol(3) + Offset(3)];
endfunction

function [List,n] = ListReduce(List,n,div),
    TmpList = [] ;
    n = int(n/div) ;
    for i = 1:n
        TmpList = [TmpList , List(:,i*div) ];
    end
    List = TmpList ;
endfunction

function [List,n,Matrix,err] = FitMatrix(List,n,Matrix,FitRadius),
    function e=G(p,z),
        V = [z(1),z(2),z(3)];
        N = [p(1), p(2), p(3);
            p(4), p(5), p(6);
            p(7), p(8), p(9)];
        e=norm(N*(V).')-FitRadius,
    endfunction
    
    guess = [1;0;0;0;1;0;0;0;1];
    
    [Sol,err]=datafit(G,List,guess);
    
    S = [Sol(1), Sol(2), Sol(3);
         Sol(4), Sol(5), Sol(6);
         Sol(7), Sol(8), Sol(9)];
         
    Matrix = S * Matrix;
    
    [List,n] = ApplyMatrix(List,n,S);
endfunction

function [List,n,Matrix,Offset,err] = FitMatrixOffset(List,n,Matrix,Offset,FitRadius),
    function e=G(p,z),
        V = [z(1),z(2),z(3)];
        N = [p(1), p(2), p(3);
             p(4), p(5), p(6);
             p(7), p(8), p(9)];
        Off = [p(10),p(11), p(12)];
        e=norm(N*(V+Off).')-FitRadius,
    endfunction
    
    guess = [1;0;0;0;1;0;0;0;1;0;0;0];
    
    [Sol,err]=datafit(G,List,guess);
    
    S = [Sol(1), Sol(2), Sol(3);
         Sol(4), Sol(5), Sol(6);
         Sol(7), Sol(8), Sol(9)];
         
    Matrix = S * Matrix;
    
    Off = [Sol(10), Sol(11), Sol(12)];
    
    [List,n] = ApplyOffset(List,n,Off)
    [List,n] = ApplyMatrix(List,n,S);
    
    Offset = [Off(1) + Offset(1) , Off(2) + Offset(2) , Off(3) + Offset(3) ];
endfunction


function [] = Display3DPoints(List,n,colorID),
    param3d(List(1,:),List(2,:),List(3,:),theta=45,alpha=45)
    e = gce();
    e.line_mode="off";
    e.mark_mode="on";
    e.mark_size=0;
//    e.mark_foreground=colorID;
//    e.mark_background=colorID;
    a = gca();
    a.isoview = "on";
endfunction

function [Var] = CalculErr(List,n,FitRadius),
    total = 0;
    for i = 1:n
        Vect = List(:,i);
        total = total + (norm(Vect)-FitRadius)^2;
    end
    Var = total / n;
endfunction
















