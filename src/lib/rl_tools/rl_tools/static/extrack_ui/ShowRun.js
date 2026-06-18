import {TrajectoryPlayer} from "./TrajectoryPlayer.js";
export class ShowRun{
    constructor(container, run, size){

        container.innerHTML = ""
        const step = Object.keys(run.steps).filter(step => run.steps[step].trajectories || run.steps[step].trajectories_compressed).sort().reverse()[0]

        const description = document.createElement("div")
        description.classList.add("show-run-description")

        const latest_run_experiment = document.createElement("span")
        latest_run_experiment.innerText = run.config.experiment
        const experiment_label = document.createElement("b")
        experiment_label.innerText = "Experiment: "
        description.appendChild(experiment_label)
        description.appendChild(latest_run_experiment)
        const latest_run_commit_hash = document.createElement("span")
        latest_run_commit_hash.innerText = run.config.commit
        const commit_label = document.createElement("b")
        description.appendChild(document.createElement("br"))
        commit_label.innerText = "Commit: "
        description.appendChild(commit_label)
        description.appendChild(latest_run_commit_hash)
        const latest_run_name = document.createElement("span")
        latest_run_name.innerText = run.config.name
        const name_label = document.createElement("b")
        description.appendChild(document.createElement("br"))
        name_label.innerText = "Name: "
        description.appendChild(name_label)
        description.appendChild(latest_run_name)
        
        // Add source label if available
        if(run.config._source){
            const latest_run_source = document.createElement("span")
            latest_run_source.innerText = run.config._source
            const source_label = document.createElement("b")
            description.appendChild(document.createElement("br"))
            source_label.innerText = "Source: "
            description.appendChild(source_label)
            description.appendChild(latest_run_source)
        }
        
        const latest_run_config = document.createElement("span")
        latest_run_config.innerText = JSON.stringify(run.config.population)
        const config_label = document.createElement("b")
        description.appendChild(document.createElement("br"))
        config_label.innerText = "Config: "
        description.appendChild(config_label)
        description.appendChild(latest_run_config)
        const latest_run_seed = document.createElement("span")
        latest_run_seed.innerText = run.config.seed
        description.appendChild(document.createElement("br"))
        const seed_label = document.createElement("b")
        seed_label.innerText = "Seed: "
        description.appendChild(seed_label)
        description.appendChild(latest_run_seed)
        const latest_run_checkpoint = document.createElement("span")
        latest_run_checkpoint.innerText = parseInt(step).toString()
        description.appendChild(document.createElement("br"))
        const checkpoint_label = document.createElement("b")
        checkpoint_label.innerText = "Checkpoint: "
        description.appendChild(checkpoint_label)
        description.appendChild(latest_run_checkpoint)

        if(run.description){
            description.appendChild(document.createElement("br"))
            const description_label = document.createElement("b")
            description_label.innerText = "Description: "
            const description_button = document.createElement("button")
            description_button.classList.add("info-button")
            description_button.innerText = "i"
            description_button.onclick = () => {
                fetch(run.description).then(response => response.text()).then(text => {
                    alert(text)
                })
            }
            description.appendChild(description_label)
            description.appendChild(description_button)
        }

        container.appendChild(description)

        const trajectory_player_container = document.createElement("div")
        // trajectory_player_container.style.height = "500px";
        const trajectory_player = new TrajectoryPlayer(run.ui_jsm, size);
        trajectory_player_container.appendChild(trajectory_player.getCanvas());
        container.appendChild(trajectory_player_container)
        if(run.steps[step].trajectories_compressed){
            trajectory_player.playTrajectories(run.steps[step].trajectories_compressed);
        }
        else{
            trajectory_player.playTrajectories(run.steps[step].trajectories);
        }

    }
}
