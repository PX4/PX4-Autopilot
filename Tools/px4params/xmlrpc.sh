python px_process_params.py

rm cookies.txt
curl --cookie cookies.txt --cookie-jar cookies.txt --user-agent Mozilla/4.0 --data "u=$XMLRPCUSER&p=$XMLRPCPASS" https://pixhawk.org/start?do=login
curl -k --cookie cookies.txt -H "Content-Type: application/xml" -X POST --data-binary @parameters.wiki "https://pixhawk.org/lib/exe/xmlrpc.php"
