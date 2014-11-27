<?php
function write_database ( $ex_status, $state=NULL, $id=NULL )
{
	$len = strlen($ex_status);

	$db = new mysqli('localhost', 'root', 'smarthouse', 'outlets');

	if ($db->connect_errno > 0) {
		die('Error connection to database: ' . $db->connect_error);
	}

	// update status
	$query = $db->prepare('UPDATE `Communication` SET `Status` = 1, `ExStatusLength` = ?, `ExtendedStatus` = ?');
	$query->bind_param('is', $len, $ex_status);
	$query->execute();

	// optionally update the state in the status table
	if (!is_null($state) && is_numeric($id)) {
		$query = $db->prepare('UPDATE `Outlets` SET `state` = ? WHERE `id` = ?');
		$query = $db->bind_param('si', $state, $id);
		$query->execute();
	}
}

function exit_with_400 ( )
{
	http_response_code(400);
	exit();
}

function get_or_die ( $key, $array )
{
	if (array_key_exists($key, $array)) {
		return $array[$key];
	}

	exit_with_400();
}

function outlet_on ( $outlet_id )
{
	$ex_status = chr($outlet_id) + chr(0x11);
	write_database($ex_status, 'on', $outlet_id);
}

function outlet_off ( $outlet_id )
{
	$ex_status = chr($outlet_id) + chr(0x22);
	write_database($ex_status, 'off', $outlet_id);
}

function outlet_get_power ( $outlet_id )
{
	$ex_status = chr($outlet_id) + chr(0x33);
	write_database($ex_status);
}

// command handlers
$commands = [
	'on' => 'outlet_on',
	'off' => 'outlet_off',
	'get_power' => 'outlet_get_power'
];

// verify the request is valid
$command = get_or_die('command', $_POST);
$outlet = get_or_die('outlet', $_POST);

// check that $outlet is numeric
if (!is_numeric($outlet)) {
	exit_with_400();
}
$outlet = intval($outlet);

// get and run the handler
$handler = get_or_die($command, $commands);
$handler('outlet');
?>
