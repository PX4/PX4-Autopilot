import createRLtoolsInterfaceBenchmark from './build/wasm_interface_benchmark.js';
import createRLtoolsInterface from './build/wasm_interface.js';

let mode = null;
let rlt = null;
let training_state = null;
let training_finished = false;
let state_dim = null;
let current_episode = null;
self.addEventListener("message", async (event) => {
    // console.log("Message received from main script: ", event.data);
    if (event.data.type === 'initialize') {
        console.log("Initializing worker, benchmark: " + event.data.payload.benchmark);
        console.assert(rlt === null)

        mode = event.data.payload.benchmark ? 'benchmark' : 'normal';
        const rlt_interface_factory = mode === "benchmark" ? createRLtoolsInterfaceBenchmark : createRLtoolsInterface;

        rlt = await rlt_interface_factory().then((rlt_emscripten) => {
            console.log("Initializing worker");
            return {
                create_training_state: rlt_emscripten._proxy_create_training_state,
                training_step: rlt_emscripten._proxy_training_step,
                destroy_training_state: rlt_emscripten._proxy_destroy_training_state,
                get_step: rlt_emscripten._proxy_get_step,
                get_evaluation_count: rlt_emscripten._proxy_get_evaluation_count,
                get_evaluation_return: rlt_emscripten._proxy_get_evaluation_return,
                get_state_dim: rlt_emscripten._proxy_get_state_dim,
                get_state_value: rlt_emscripten._proxy_get_state_value,
                get_episode: rlt_emscripten._proxy_get_episode,
                get_episode_return: rlt_emscripten._proxy_get_episode_return,
            }
        })

        self.postMessage({id: event.data.id, type: 'worker_initialization_finished'});
        return
    }
    if (event.data.type === 'initialize_training_state') {
        console.assert(rlt !== null)
        let seed = event.data.payload && event.data.payload.seed ? event.data.payload.seed : 0;
        console.log("Using seed: " + seed)
        training_state = rlt.create_training_state(seed);
        state_dim = rlt.get_state_dim();
        current_episode = 1;
        self.postMessage({id: event.data.id, type: 'finished_initializing_training_state'});
        return
    }
    if (event.data.type === 'full_training') {
        console.assert(training_state !== null)
        while(!training_finished) {
            let step = rlt.get_step(training_state);
            if (step % 100 === 0) {
                self.postMessage({type: 'training_step', payload: {step: step}});
            }
            training_finished = rlt.training_step(training_state);
        }
        if(mode !== 'benchmark') {
            let evaluation_returns = []
            let num_evaluations = rlt.get_evaluation_count();
            for(let i = 0; i < num_evaluations; i++){
                let evaluation_return = rlt.get_evaluation_return(training_state, i);
                evaluation_returns.push(evaluation_return);
            }
            self.postMessage({id: event.data.id, type: 'finished_training', payload: {evaluation_returns: evaluation_returns}});
        }
        else{
            self.postMessage({id: event.data.id, type: 'finished_training'});
        }
        return
    }
    if (event.data.type === 'train_one_step') {
        console.assert(training_state !== null)
        console.assert(!training_finished)
        let step = rlt.get_step(training_state);
        training_finished = rlt.training_step(training_state);
        let state = []
        for(let i = 0; i < state_dim; i++){
            let state_value = rlt.get_state_value(training_state, 0, i);
            state.push(state_value);
        }
        let episode = rlt.get_episode(training_state, 0);
        let episode_return = null;
        if(episode !== current_episode){
            current_episode = episode;
            episode_return = rlt.get_episode_return(training_state, 0, current_episode - 2); // -2 because it always points to the next episode (-1) and because it has already finished the episode of interest (-1)
        }

        self.postMessage({
            id: event.data.id,
            type: 'train_one_step',
            payload: {
                step: step,
                training_finished: training_finished,
                state: state,
                episode: current_episode,
                episode_return: episode_return
            }
        });
        return
    }
    if (event.data.type === 'reset_training') {
        console.assert(training_state !== null)
        rlt.destroy_training_state(training_state);
        training_state = rlt.create_training_state();
        self.postMessage({id: event.data.id, type: 'finished_reset'});
        return
    }
    if (event.data.type === 'destroy_training_state') {
        console.assert(training_state !== null)
        rlt.destroy_training_state(training_state);
        training_state = null;
        training_finished = false;
        state_dim = null;
        current_episode = null;
        self.postMessage({id: event.data.id, type: 'finished_destroying_training_state'});
        return
    }
    console.warn("Unknown message type: " + event.data.type)
})
