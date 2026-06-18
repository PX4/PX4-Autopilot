import createRLtoolsInterface from './build/wasm_interface.js';

let mode = null;
let rlt = null;
let training_state = null;
let training_finished = false;
let state_dim = null;
let current_episode = null;

async function async_main(){
    const rlt = await createRLtoolsInterface()
    console.log("Initializing worker");
    let training_state = rlt._proxy_create(1);
    console.log("Training state: ", training_state);

    let messages = []
    let playbackSpeed = 1;
    self.addEventListener("message", async (event) => {
        if(event.data.channel === "setPlaybackSpeed"){
            playbackSpeed = event.data.data;
            console.log("Setting playback speed to: ", playbackSpeed);
        }
        else{
            messages.push(event.data)
        }
    })


    let main = async () =>{
        while(true){
            let sleep = 0;
            while(messages.length > 0){
                const message = messages.shift();
                const message_string = JSON.stringify(message)
                let message_ptr = rlt.stringToNewUTF8(message_string);
                sleep += rlt._proxy_step_message(training_state, message_ptr)
            }
            for(let i = 0; i < 1; i++){
                sleep += rlt._proxy_step(training_state)
            }

            let pop_messages = await new Promise((resolve, reject) => {
                let interval_timer = null;
                let pop_message = () => {
                    if(rlt._proxy_num_messages(training_state) > 0){
                        const message_pointer = rlt._proxy_pop_message(training_state);
                        const message = rlt.UTF8ToString(message_pointer);
                        rlt._proxy_delete_message(message_pointer)
                        self.postMessage(JSON.parse(message))
                        // await new Promise(resolve => setTimeout(resolve, 10/playbackSpeed));
                    }
                    else{
                        clearInterval(interval_timer);
                        resolve();
                    }
                }
                interval_timer = setInterval(pop_message, 10/playbackSpeed);
            })
        }
    }
    main()
}
async_main()
    // console.log("Message received from main script: ", event.data);
