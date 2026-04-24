let opts = {
	title: "Wireless sensor data",
	id: "chart1",
	class: "mychart",
	width: window.innerWidth*0.7,
	height: window.innerHeight*0.8,
}

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
		})
	setColors()
	let data = [[]]
	
	const socket = new WebSocket("/stream")
	let plot = new uPlot(opts, data, document.getElementById("chart1"))
	let wt = false
	socket.addEventListener("message", (evt) => {
		data = tail(data, evt.data)
		printStats(data)
		plot.setData(data)
	})
	await new Promise(r => requestAnimationFrame(r));//For production
}
loop();

function tail(data, evt){
	let hist_len = document.getElementById("n_hist").value - 0
	let nd = JSON.parse(evt)
	if (data.length > 1){
		for (i = 0; i < data.length; i++){
			data[i] = data[i].concat(nd[i])
			j = Math.max(0, data[i].length-hist_len)
			data[i] = data[i].slice(j)
		}
	} else {
		data = nd
	}
	return data;
}

function pretty(matrix) {
	// Seperates the lines with newline instead of brackets
	return matrix.map(row => row+'\r\n');
}

function save_recording(data) {
	file = new Blob(pretty(data), {type:"octet/stream"});
	tag = document.createElement("li");
	tag.innerHTML = '<a href="'+URL.createObjectURL(file)+'" download="acceleration_log.csv">Download</a>';
	document.getElementById("dl").appendChild(tag);
}

async function get_fragment(id, url) {
	p = document.getElementById(id)
	await fetch(url).then(response => response.text()).then(html => p.innerHTML=html)
}

get_fragment('logs', '/logs')