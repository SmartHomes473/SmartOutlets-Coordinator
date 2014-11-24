<?php

$db = new mysqli("localhost", "root", "smarthouse", "outlets");

if ($db->connect_errno > 0) {
	die("Error connection to database: " . $db->connect_error);
}

// SELECT all outlets
$query = $db->prepare("SELECT * FROM outlet_table;");
$query->execute();
$result = $query->get_result();

// construct outlet list
$outlets = array();
while ($row = $result->fetch_array()) {
	$outlets[] = $row;
}
?>
<!DOCTYPE html>
<html>
<head>
	<title>Outlets Control!</title>
	<link rel="stylesheet" href="css/outlets.css">
</head>

<body>

	<div id="outlets">

<?php
	// print out a row for each outlet
	foreach ($outlets as $outlet) {
		$row_html = <<<EOHTML
<div id="outlet_{$outlet['id']}" class="outlet-row" data-id="{$outlet['id']}" data-state="{$outlet['state']}">
EOHTML;

		echo $row_html;
	}
?>

	</div>

	<script src="http://ajax.googleapis.com/ajax/libs/jquery/1/jquery.min.js"></script>
	<script src="js/outlets.js"></script>
</body>
</html>
