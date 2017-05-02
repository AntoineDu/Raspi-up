<?php
error_reporting(E_ALL);
shell_exec('stty -F /dev/ttyAMA0 -isig');
shell_exec('stty -F /dev/ttyAMA0 -icanon');
function microtime_float()
{
    list($usec, $sec) = explode(" ", microtime());
    return ((float) $usec + (float) $sec);
}
include "php_serial.class.php";
$SerialCom = new phpSerial;
$SerialCom->deviceSet("/dev/ttyAMA0");
$SerialCom->confBaudRate(19200);
$SerialCom->confParity("none");
$SerialCom->confCharacterLength(8);
$SerialCom->confStopBits(1);
$SerialCom->deviceOpen();
$Un = "$";
$Deux = $_POST['dropdown15'];
$Trois = $_POST['text2'];
$Trois = chr($Trois);
$Quatre = $_POST['text8'];
$Quatre = chr($Quatre);
$abcd = $Un . $Deux . $Trois . $Quatre;
$SerialCom->sendMessage($abcd);
$read = '';
$theResult = '';
$start = microtime_float();
while ( ($read == '') && (microtime_float() <= $start + 0.5) ) {
    $read = $SerialCom->readPort();
    if ($read != '') {
        $theResult .= $read;
        $read = '';
    }
}
$SerialCom->deviceClose();
$form->data['text17'] = $theResult;
?>