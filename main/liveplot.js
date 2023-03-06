const opts = {
	title: "Acceleration",
	id: "chart1",
	class: "mychart",
	width: window.innerWidth*0.7,
	height: window.innerHeight*0.8,
	series: [
		{
			scale: "seconds",
			label: "seconds",
		},{
			scale: "acceleration",
			label: "X",
			stroke:"red",
			width: 1,
		},{
			scale: "acceleration",
			label: "Y",
			stroke:"green",
			width: 1,
		},{
			scale: "acceleration",
			label: "Z",
			stroke:"blue",
			width: 1,
		},
	],
	scales: {
		"seconds": {
			time: false,
		},
		"acceleration": {
			auto: true,
		},
	},
}
async function getData(t0){
	return fetch("/data/"+t0)
		.then((response) => response.json())
		.then((data) => {
			t = [];
			n = data.x.length;
			for(let i = 0; i<n; ++i){
				t.push(i+data.t0);
			}
			return [t,data.x,data.y,data.z];
	});
}

var stats;

function printStats(data){
	axes = ['X','Y','Z'];
	let s = '';
	stats = data.slice(1,4).map(a => [Math.min(...a),Math.max(...a)]);
	for(let i=0; i<3; ++i){
		s += axes[i]+'_Min:'+stats[i][0]+"<br>";
		s += axes[i]+'_Max:'+stats[i][1]+"<br>";
	}
	let e = document.getElementById("stats");
	e.innerHTML = s;
}

function autorange(){
	let x_min = stats[0][0];
	let x_max = stats[0][1];
	margin = 0.1 * (x_max - x_min);
	document.getElementById("minv").value = x_min - margin;
	document.getElementById("maxv").value = Number(x_max) + margin;
}


async function loop(){
	let data = await getData(0);
	let plot = new uPlot(opts, data, document.getElementById("chart1"));
	for(;;){
		let t0 = Number(data[0].slice(-1))+1;
		console.log(t0);
		let newdata = await getData(t0);
		for(let i = 0; i < data.length; ++i){
			let a = data[i].concat(newdata[i]);
			data[i] = a.slice(-6000,-1);
		}
		printStats(data);
		plot.setData(data);
		logData(newdata);
		await new Promise(r => requestAnimationFrame(r));//For production
	}
}
loop();

function transpose(matrix) {
	return matrix[0].map((col, i) => matrix.map(row => row[i]));
}

function pretty(matrix) {
	// Seperates the lines with newline instead of brackets
	return matrix.map(row => row+'\r\n');
}

function save_recording(data) {
	file = new Blob(pretty(transpose(data)), {type:"octet/stream"});
	tag = document.createElement("li");
	tag.innerHTML = '<a href="'+URL.createObjectURL(file)+'" download="acceleration_log.csv">Download</a>';
	document.getElementById("dl").appendChild(tag);
}

var logged=[[],[],[],[]];
let timeout=0;
function logData(newdata){
	enabled = document.getElementById("en").checked;
	notEmpty = logged[0].length > 0;
	//For now, only works with X axis
	if(enabled){
		if((Math.min(...newdata[1])<document.getElementById("minv").value)
		 || (Math.max(...newdata[1])>document.getElementById("maxv").value)){
			 timeout=0;
		 } else {
			 ++timeout;
		 }
		if(notEmpty && (timeout > document.getElementById("timeout").value)){
			save_recording(logged);
			logged=[[],[],[],[]];
		} else {
			for(let i = 0; i < newdata.length; ++i){
				let a = logged[i].concat(newdata[i]);
				logged[i] = a.slice(-6000,-1);
			}
		}
	} else if(notEmpty){
		save_recording(logged);
		logged=[[],[],[],[]];
	}
}

