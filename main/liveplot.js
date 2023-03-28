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

var stats;

function printStats(data){
	let axes = ['X','Y','Z'];
	let s = '';
	stats = data.slice(1,4).map(a => [Math.min(...a),Math.max(...a)]);
	for(let i=0; i<3; ++i){
		s += axes[i]+'_Min:'+stats[i][0]+"<br>";
		s += axes[i]+'_Max:'+stats[i][1]+"<br>";
	}
	let e = document.getElementById("stats");
	e.innerHTML = s;
}

let axes = ["X accel","Y accel","Z accel"]
function set_axes_options(){
    let e = document.getElementById("axis")
    for( a of axes ){
        let n = document.createElement("option")
        n.innerHTML = a
        e.appendChild(n)
    }
}
function get_axes(){
    let e = document.getElementById("axis")
    return axes.findIndex((v) => e.value == v)
}
set_axes_options();

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

/**
 * This transformer takes binary Uint8Array chunks from a `fetch`
 * and translates them to chunks of strings.
 *
 * @implements {TransformStreamTransformer}
 */
class Uint8ArrayToJsonTransformer {
  constructor() {
    this.decoder = new TextDecoder()
    this.lastString = ''
  }

  /**
   * Receives the next Uint8Array chunk from `fetch` and transforms it.
   *
   * @param {Uint8Array} chunk The next binary data chunk.
   * @param {TransformStreamDefaultController} controller The controller to enqueue the transformed chunks to.
   */
  transform(chunk, controller) {
    //console.log('Received chunk %o with %d bytes.', chunk, chunk.byteLength)

    // Decode the current chunk to string and prepend the last string
    const string = `${this.lastString}${this.decoder.decode(chunk)}`

    // Extract lines from chunk
    const lines = string.split(/\r\n|[\r\n]/g)

    // Save last line, as it might be incomplete
    this.lastString = lines.pop() || ''

    // Enqueue each line in the next chunk
    for (const line of lines) {
      controller.enqueue(JSON.parse(line).data)
    }
  }

  /**
   * Is called when `fetch` has finished writing to this transform stream.
   *
   * @param {TransformStreamDefaultController} controller The controller to enqueue the transformed chunks to.
   */
  flush(controller) {
    // Is there still a line left? Enqueue it
    if (this.lastString) {
      controller.enqueue(this.lastString)
    }
  }
}


async function loop(){

    // Create a transform stream with our transformer
    const ts = new TransformStream(new Uint8ArrayToJsonTransformer())
    // Fetch the text file
    const response = await fetch('/stream')
    // Get a ReadableStream on the text file's body
    const rs = response.body
    // Apply our Transformer on the ReadableStream to create a stream of strings
    const lineStream = rs.pipeThrough(ts)
    // Read the stream of strings
    const stream = lineStream.getReader()
  
    let data = await stream.read().then((d) => d.value)
	let plot = new uPlot(opts, tail(data), document.getElementById("chart1"))
    let wt = false
	for(;;){
        await stream.read().then((d) => d.value)
            .then((newdata) => {
            data = data.concat(newdata)
            let p = tail(data)
            printStats(p)
            get_trigger()
            plot.setData(p)
            let nt = get_trigger()
            if(nt && !wt){
                data = data.slice(-6000,-1)
            } else if(wt && !nt){
                save_recording(data) // Will include 6 seconds on either side of the trigger
            }
            wt = nt
        })
		await new Promise(r => requestAnimationFrame(r));//For production
	}
}
loop();

function tail(data){
    vals = data.slice(-6000,-1);
    let t = [];
    for (let i = data.length - vals.length; i < data.length; ++i){
        t.push(i*0.01);
    }
    t = [t].concat(transpose(vals));
    return t;
}

function transpose(matrix) {
	return matrix[0].map((col, i) => matrix.map(row => row[i]));
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

