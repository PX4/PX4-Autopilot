//[List,n]=Load(pathname);
//[List,n] = DeleteZeros(List,n);
//[List,n] = ApplyMatrix(List,n,Matrix);
//[List,n] = ApplyOffset(List,n,Offset);
//[List,n,Offset] = CenterMinMax(List,n,Offset);
//[List,Offset] = CenterFit(List,Offset);
//[List,n] = ListReduce(List,n,div);
//[List,n,Matrix,err] = FitMatrix(List,n,Matrix,FitRadius);
//[List,n,Matrix,Offset,err] = FitMatrixOffset(List,n,Matrix,Offset,FitRadius);
//[] = Display3DPoints(List,n);
//[Var] = CalculErr(List,n,FitRadius)


pathname = fullpath(get_absolute_file_path('EllipsoidMain.sce')+"EllipsoidAnalysis.sce");

exec(pathname);

pathname = fullpath(get_absolute_file_path('EllipsoidMain.sce')+"Calib_PX4_ID02_v3");

Offset = [0, 0, 0];
Matrix = [1,0,0;0,1,0;0,0,1];

// Intensity of local magnetic field
FitRadius = 0.46;

// load set of measurements
[List,n] = Load(pathname);
ListSave = List;

[List,n] = DeleteZeros(List,n);

[Var] = CalculErr(List,n,FitRadius);

[List2,n2,Offset] = CenterMinMax(List,n,Offset);

// divide by 9 the number of points in the list
[List2,n2] = ListReduce(List2,n2,9);


[List2,n2,Matrix,Offset,err] = FitMatrixOffset(List2,n2,Matrix,Offset,FitRadius);

[List3,n3] = ApplyOffset(List,n,Offset);
[List4,n4] = ApplyMatrix(List3,n3,Matrix);

[Var3] = CalculErr(List3,n3,FitRadius);
[Var4] = CalculErr(List4,n4,FitRadius);
//
//Display3DPoints(List3,3);
//Display3DPoints(List3,4);
