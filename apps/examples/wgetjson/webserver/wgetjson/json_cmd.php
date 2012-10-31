<?php
/*
$json = "{\n\"name\": \"Jack (\\\"Bee\\\") Nimble\", \n\"format\": {\"type\":       \"rect\", \n\"width\":      1920, \n\"height\":     1080, \n\"interlace\":  false,\"frame rate\": 24\n}\n}";
*/
$ja["name"] = "Jack (\"Bee\") Nimble";
$ja["format"]["type"] = "rect"; 
$ja["format"]["width"] = 1920;
$ja["format"]["height"] = 1080;
$ja["format"]["interlace"] = false;
$ja["format"]["frame rate"] = 24;

echo json_encode($ja);
?>