(function ( ) {
	'use strict';

	var $outlets =  $('.outlet-row');

	console.log('hi');

	// populate the table
	$('.outlet-row').each(function() {
		var $this = $(this),
			id,
			state;

		id = $this.data('id');
		state = $this.data('state');

		// add label
		$this.append($(
			'<div class="outlet-name">' +
			'Outlet ' + id +
			'</div>'
		));

		// add toggle button
		$this.append($(
			'<div class="outlet-button outlet-state-' + state + '">' + state + '</div>'
		));
	});

	// attach on-click handlers
	$('.outlet-button').click(function() {
		var $this = $(this),
			$parent = $this.parent(),
			id,
			state;

		id = $parent.data('id');
		state = $parent.data('state');

		// toggle the state
		state = state === 'off' ? 'on' : 'off';
		$this.text(state);
		$parent.data('state', state);

		// change the class
		$this.toggleClass('outlet-state-off');
		$this.toggleClass('outlet-state-on');

		// debug
		console.log('turned outlet ' + id + ' ' + state);
	});
}());
