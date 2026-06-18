import * as hdf5 from "jsfive";
import * as math from 'mathjs'

class Matrix{
    constructor(dataset){
        this.rows = dataset.shape[0]
        this.cols = dataset.shape[1]
        const data_flat = math.matrix(dataset.value)
        this.data = math.reshape(data_flat, [this.rows, this.cols])
    }
}

class Tensor{
    constructor(dataset){
        this.shape = dataset.shape
        const data_flat = math.matrix(dataset.value)
        this.data = math.reshape(data_flat, this.shape)
    }
}

class StandardizeLayer{
    constructor(group){
        this.mean = group.get("mean").attrs.type === "matrix" ? new Matrix(group.get("mean").get("parameters")) : new Tensor(group.get("mean").get("parameters"))
        this.dim = this.mean.shape.length == 2 ? this.mean.shape[1] : this.mean.shape[0]
        this.input_shape = [null, null, this.dim]
        this.output_shape = [null, null, this.dim]
        this.precision = group.get("precision").attrs.type === "matrix" ? new Matrix(group.get("precision").get("parameters")) : new Tensor(group.get("precision").get("parameters"))
    }
    description(){
        return `Standardize(${this.output_shape[2]})`
    }
    evaluate(input){
        const leading_dimension = input.size().slice(0, -1).reduce((a, b) => a * b, 1)
        const input_reshaped = math.reshape(input, [leading_dimension, input.size()[input.size().length - 1]])
        let [output, state] = this.evaluate_step(input_reshaped)
        const output_shape = input.size().slice(0, -1).concat(this.dim)
        output = math.reshape(output, output_shape)
        return output
    }
    evaluate_step(input, state){
        let output = math.dotMultiply(math.subtract(input, this.mean.data), this.precision.data)
        return [output, null]
    }
}

class DenseLayer{
    constructor(group){
        this.weights = group.get("weights").attrs.type === "matrix" ? new Matrix(group.get("weights").get("parameters")) : new Tensor(group.get("weights").get("parameters"))
        this.input_shape = [null, null, this.weights.shape[1]]
        this.output_shape = [null, null, this.weights.shape[0]]
        this.biases = group.get("biases").attrs.type === "matrix" ? new Matrix(group.get("biases").get("parameters")) : new Tensor(group.get("biases").get("parameters"))
        this.dim = this.biases.shape.length == 2 ? this.biases.shape[1] : this.biases.shape[0]
        this.activation_function_name = group.attrs.activation_function
    }
    description(){
        return `Dense(${this.output_shape[2]})`
    }
    activation_function(input){
        if(this.activation_function_name === "IDENTITY"){
            return input
        }
        else if(this.activation_function_name === "RELU"){
            return math.map(input, x => x > 0 ? x : 0)
        }
        else if(this.activation_function_name === "SIGMOID"){
            return math.map(input, x => 1 / (1 + math.exp(-x)))
        }
        else if(this.activation_function_name === "TANH"){
            return math.map(input, x => math.tanh(x))
        }
        else if(this.activation_function_name === "FAST_TANH"){
            function clamp(value, min, max) {
                return Math.max(min, Math.min(max, value));
            }
            return math.map(input, inp => {
                const x = clamp(inp, -3.0, 3.0);
                const x_squared = x * x;
                return x * (27 + x_squared) / (27 + 9 * x_squared);
            })
        }
        else{
            console.error("Unknown activation function: ", this.activation_function_name)
            return null
        }
    }
    evaluate(input){
        const leading_dimension = input.size().slice(0, -1).reduce((a, b) => a * b, 1)
        const input_reshaped = math.reshape(input, [leading_dimension, input.size()[input.size().length - 1]])
        let [output, state] = this.evaluate_step(input_reshaped)
        const output_shape = input.size().slice(0, -1).concat(this.dim)
        output = math.reshape(output, output_shape)
        return output
    }
    evaluate_step(input, state){
        let output = math.multiply(this.weights.data, math.transpose(input))
        output = math.add(math.transpose(output), this.biases.data)
        output = this.activation_function(output)
        return [output, null]
    }
}

class GRULayer{
    constructor(group){
        this.weights_hidden = new Tensor(group.get("weights_hidden").get("parameters"))
        this.weights_input = new Tensor(group.get("weights_input").get("parameters"))
        this.hidden_dim = Math.floor(this.weights_input.shape[0] / 3)
        this.input_shape = [null, null, this.weights_input.shape[1]]
        this.output_shape = [null, null, this.hidden_dim]
        this.biases_hidden = new Tensor(group.get("biases_hidden").get("parameters"))
        this.biases_input = new Tensor(group.get("biases_input").get("parameters"))
        this.initial_hidden_state = new Tensor(group.get("initial_hidden_state").get("parameters"))
    }
    description(){
        return `GRU(${this.hidden_dim})`
    }
    reset(){
        return null
    }
    evaluate(input){
        if(input.size().length === 2){
            const [output, state] = this.evaluate_step(input)
            return output
        }
        else{
            const [SEQUENCE_LENGTH, BATCH_SIZE, INPUT_SIZE] = input.size()
            let state = null
            const outputs = []
            for(let i = 0; i < SEQUENCE_LENGTH; i++){
                const step_input = math.matrix((math.subset(input, math.index(i, math.range(0, BATCH_SIZE), math.range(0, INPUT_SIZE)))).toArray()[0])
                const [output, new_state] = this.evaluate_step(step_input, state)
                outputs.push(output)
                state = new_state
            }
            return math.matrix(outputs)
        }
    }
    evaluate_step(input, state){
        const [BATCH_SIZE, INPUT_SIZE] = input.size()
        console.assert(INPUT_SIZE === this.weights_input.shape[1], "Input size does not match weights size")
        if(state === null){
            state = math.matrix((new Array(BATCH_SIZE)).fill(0).map(() => this.initial_hidden_state.data))
        }
        console.assert(state.size()[0] === BATCH_SIZE, "State size does not match input size")
        const Wh = math.transpose(math.multiply(this.weights_hidden.data, math.transpose(state)))
        const Wi = math.transpose(math.multiply(this.weights_input.data, math.transpose(input)))
        const Wh_rz = math.subset(Wh, math.index(math.range(0, BATCH_SIZE), math.range(0, this.hidden_dim * 2)))
        const Wh_n = math.subset(Wh, math.index(math.range(0, BATCH_SIZE), math.range(this.hidden_dim * 2, this.hidden_dim * 3)))
        const Wi_rz = math.subset(Wi, math.index(math.range(0, BATCH_SIZE), math.range(0, this.hidden_dim * 2)))
        const Wi_n = math.subset(Wi, math.index(math.range(0, BATCH_SIZE), math.range(this.hidden_dim * 2, this.hidden_dim * 3)))
        const bh_rz = math.subset(this.biases_hidden.data, math.index(math.range(0, this.hidden_dim * 2)))
        const bh_n = math.subset(this.biases_hidden.data, math.index(math.range(this.hidden_dim * 2, this.hidden_dim * 3)))
        const bi_rz = math.subset(this.biases_input.data, math.index(math.range(0, this.hidden_dim * 2)))
        const bi_n = math.subset(this.biases_input.data, math.index(math.range(this.hidden_dim * 2, this.hidden_dim * 3)))
        const rz_pre = math.add(math.add(math.add(Wh_rz, Wi_rz), bh_rz), bi_rz)
        const sigmoid = (x) => 1 / (1 + Math.exp(-x))
        const rz = math.map(rz_pre, sigmoid)
        const r = math.subset(rz, math.index(math.range(0, BATCH_SIZE), math.range(0, this.hidden_dim)))
        const z = math.subset(rz, math.index(math.range(0, BATCH_SIZE), math.range(this.hidden_dim, this.hidden_dim * 2)))
        const n_pre_pre = math.add(Wh_n, bh_n)
        const n_pre = math.add(math.add(math.dotMultiply(r, n_pre_pre), Wi_n), bi_n)
        const n = math.map(n_pre, Math.tanh)
        const new_state = math.add(math.dotMultiply(z, state), math.dotMultiply(math.subtract(1, z), n))
        return [new_state, new_state]
    }
}
class SampleAndSquashLayer{
    constructor(group){
        this.input_shape = [null, null, null]
        this.output_shape = [null, null, null]
    }
    description(){
        return `SampleAndSquash`
    }
    evaluate(input){
        const mean = math.subset(input, math.index(
            ...input.size().map((x, i) => {
                if(i === input.size().length - 1){
                    return math.range(0, x/2)
                }
                else{
                    return math.range(0, x)
                }
            })
        ));
        return math.map(mean, Math.tanh)
    }
    evaluate_step(input, state){
        return [this.evaluate(input), null]
    }
}
class MLP{
    constructor(group){
        this.input_layer = new DenseLayer(group.get("input_layer"))
        this.hidden_layers = []
        for(let i = 0; i < group.attrs.num_layers - 2; i++){
            this.hidden_layers.push(new DenseLayer(group.get(`hidden_layer_${i}`)))
        }
        this.output_layer = new DenseLayer(group.get("output_layer"))
        this.input_shape = this.input_layer.input_shape
        this.output_shape = this.output_layer.output_shape
    }
    description(){
        return `MLP(${this.input_layer.description()}, ${this.hidden_layers.map(layer => layer.description()).join(", ")}, ${this.output_layer.description()})`
    }
    evaluate(input){
        let current = this.input_layer.evaluate(input)
        for(let i = 0; i < this.hidden_layers.length; i++){
            const layer = this.hidden_layers[i]
            current = layer.evaluate(current)
        }
        current = this.output_layer.evaluate(current)
        return current
    }
    evaluate_step(input, state){
        return [this.evaluate(input), null]
    }
}

class Sequential{
    constructor(group){
        this.layers = []
        for(let i = 0; i < group.get("layers").keys.length; i++){
            this.layers.push(layer_dispatch(group.get("layers").get(`${i}`)))
        }
        this.input_shape = this.layers[0].input_shape
        this.output_shape = this.layers.slice().reverse().find(layer => layer.output_shape.reduce((a, c) => (a || c !== null), null)).output_shape
    }
    description(){
        return `Sequential(${this.layers.map(layer => layer.description()).join(", ")})`
    }
    reset(){
        return this.layers.map(layer => {
            return layer.reset ? layer.reset() : null
        })
    }
    evaluate(input){
        let current = input
        for(let i = 0; i < this.layers.length; i++){
            const layer = this.layers[i]
            if(layer){
                current = layer.evaluate(current)
            }
        }
        return current
    }
    evaluate_step(input, state){
        if(!state){
            state = this.reset()
        }
        let current = input
        const new_state = []
        for(let i = 0; i < this.layers.length; i++){
            const layer = this.layers[i]
            const layer_state = state[i]
            if(layer){
                const [new_current, new_layer_state] = layer.evaluate_step(current, layer_state)
                current = new_current
                new_state.push(new_layer_state)
            }
            else{
                new_state.push(null)
            }
        }
        return [current, new_state]
    }
}


function layer_dispatch(group){
    let model = null
    if(group.attrs.type === "dense") {
        model = new DenseLayer(group)
    }
    else if(group.attrs.type === "gru") {
        model = new GRULayer(group)
    }
    else if(group.attrs.type === "mlp") {
        model = new MLP(group)
    }
    else if(group.attrs.type === "sequential") {
        model = new Sequential(group)
    }
    else if(group.attrs.type === "sample_and_squash") {
        model = new SampleAndSquashLayer(group)
    }
    else if(group.attrs.type === "standardize") {
        model = new StandardizeLayer(group)
    }
    else{
        console.error("Unknown layer type: ", group.attrs.type)
        model = null
    }
    model.checkpoint_name = null
    if("checkpoint_name" in group.attrs){
        model.checkpoint_name = group.attrs.checkpoint_name
    }
    else{
    }
    model.meta = null
    if("meta" in group.attrs){
        model.meta = JSON.parse(group.attrs.meta)
    }
    return model
}

function load_from_array_buffer(buffer){
    var f = new hdf5.File(buffer, "");
    const model = layer_dispatch(f.get("actor"))
    const input = new Tensor(f.get("example").get("input"))
    const target_output = new Tensor(f.get("example").get("output"))

    const output = model.evaluate(input.data)

    const diff = math.subtract(output, target_output.data)
    const diff_reduce = math.flatten(diff).valueOf().reduce((a, c) => a + Math.abs(c)) / diff.size().reduce((a, c) => a * c, 1)
    console.log("Example diff per element: ", diff_reduce)
    console.assert(diff_reduce < 1e-6, "Output is not close enough to target output")
    return model
}

export function load(input) {
    if(typeof input === "string"){
        return fetch(input)
            .then(function(response) {
                return response.arrayBuffer()
            })
            .then(load_from_array_buffer);
    }
    else if(input instanceof ArrayBuffer){
        return load_from_array_buffer(input)
    }
    else{
        console.error("Input is not a string or ArrayBuffer")
        return null
    }
}

