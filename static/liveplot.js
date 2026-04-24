let opts = {
	title: "Wireless sensor data",
	id: "chart1",
	class: "mychart",
	width: window.innerWidth*0.7,
	height: window.innerHeight*0.8,
}

let sample_rate = 1000;
let data = [[], [], [], []];
let allData = [[], [], [], []];

function setColors(){
	const styles = [{stroke: "red", width:2},
			{stroke:"blue", width:4},
			{stroke:"black", width:1, dash:[10,5]},
			{stroke:"green", width:2, dash:[2,1]},
			{stroke:"#f0f", width:1.5, dash:[5,2,1]},
	]
	for(let i=1; i<opts.series.length; ++i){
		opts.series[i] = Object.assign(opts.series[i], styles[i-1])
	}
}

var stats;

function printStats(data){
	let labels = opts.series.map(x => x.label).slice(1)
	let s = '';
	stats = data.slice(1).map(a => [Math.min(...a),Math.max(...a)]);
	for(let i=0; i<labels.length; ++i){
		s += labels[i]+' min:'+stats[i][0]+"<br>";
		s += labels[i]+' max:'+stats[i][1]+"<br>";
	}
	let e = document.getElementById("stats");
	e.innerHTML = s;
}

function get_axes(){
	let e = document.getElementById("axis")
	return axes.findIndex((v) => e.value == v)
}

function autorange(){
	let axis = get_axes()
	let minv = stats[axis][0]
	let maxv = stats[axis][1]
	margin = 0.1 * (maxv - minv)
	document.getElementById("minv").value = minv - margin
	document.getElementById("maxv").value = Number(maxv) + margin
}

let trig_timer = 0
function get_trigger(){
	let trg = document.getElementById("trg")
	let enabled = document.getElementById("en").checked
	let axis = get_axes()
	let minv = document.getElementById("minv").value
	let maxv = document.getElementById("maxv").value
	let timeout = document.getElementById("timeout").value * 1000
	if( !enabled ){
		trg.checked = false
	} else if((minv < stats[axis][0]) && maxv > stats[axis][1]){
		if((Date.now() - trig_timer) >  timeout){
			trg.checked = false
		}
	} else {
		trg.checked = true
		trig_timer = Date.now()
	}
	return trg.checked
}


async function loop(){
	await fetch("/header")
		.then(response => response.json())
		.then(header => {
			opts.scales = header.scales
			opts.series = header.series
			if (header.sample_rate) sample_rate = header.sample_rate;
		})
	setColors()
	data = [[], [], [], []]
	allData = [[], [], [], []]

	const socket = new WebSocket("/stream")
	socket.binaryType = 'arraybuffer';
	let plot = new uPlot(opts, data, document.getElementById("chart1"))
	let wt = false
	socket.addEventListener("message", (evt) => {
		data = appendBinary(data, evt.data)
		printStats(data)
		plot.setData(data)
	})
	await new Promise(r => requestAnimationFrame(r));//For production
}
loop();

function appendBinary(data, buffer) {
	let view = new DataView(buffer);
	if (view.byteLength < 6) return data;
	let t0 = view.getFloat32(0, true);
	let n = view.getUint16(4, true);
	if (view.byteLength < 6 + n * 6) return data;
	let hist_len = document.getElementById("n_hist").value - 0;
	let dt = 1.0 / sample_rate;
	for (let i = 0; i < n; i++) {
		const t = t0 + i * dt;
		const x = view.getInt16(6 + i * 6,     true);
		const y = view.getInt16(6 + i * 6 + 2, true);
		const z = view.getInt16(6 + i * 6 + 4, true);
		data[0].push(t); data[1].push(x); data[2].push(y); data[3].push(z);
		allData[0].push(t); allData[1].push(x); allData[2].push(y); allData[3].push(z);
	}
	let excess = data[0].length - hist_len;
	if (excess > 0) {
		for (let k = 0; k < data.length; k++) {
			data[k] = data[k].slice(excess);
		}
	}
	return data;
}

function save_recording() {
	const labels = opts.series.map(s => s.label);
	const rows = [labels.join(',')];
	for (let i = 0; i < allData[0].length; i++) {
		rows.push(allData.map(col => col[i]).join(','));
	}
	const csv = rows.join('\r\n');
	const file = new Blob([csv], {type: 'text/csv'});
	const tag = document.createElement('li');
	tag.innerHTML = '<a href="' + URL.createObjectURL(file) + '" download="acceleration_log.csv">Download</a>';
	document.getElementById('dl').appendChild(tag);
}