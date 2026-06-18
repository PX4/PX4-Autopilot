import {parseIndex} from "./ParseIndex.js";
import {TrajectoryPlayer} from "./TrajectoryPlayer.js";

class Spoiler{
    constructor(parent, summary_text, terminal, on_open, on_close){
        this.spoiler = document.createElement('details');
        this.spoiler.classList.add("experiment-spoiler");
        this.spoiler.classList.add("spoiler");
        this.summary = document.createElement('summary');
        this.summary.classList.add("experiment-summary");
        this.summary.innerHTML = summary_text;
        this.spoiler.appendChild(this.summary);
        this.terminal = terminal;
        if(!this.terminal){
            this.child_list = document.createElement('ul');
            this.spoiler.appendChild(this.child_list)
        }
        parent.appendChild(this.spoiler);
        this.spoiler.addEventListener('toggle', () => {
            if(this.spoiler.open){
                if(on_open){
                    on_open(this);
                }
            }
            else{
                if(on_close){
                    on_close(this);
                }
            }
        })
    }
    appendChild(child){
        if(this.terminal){
            throw "Cannot append children to terminal spoiler";
        }
        this.child_list.appendChild(child);
    }
    setContent(content){
        if(!this.terminal){
            throw "Cannot set content to non terminal spoiler";
        }
        this.spoiler.appendChild(content);
    }
}

export class ExplorerStep{
    constructor(parent, run, step, options){
        this.options = options || {};
        this.run = run
        this.step = step

        this.content = document.createElement('div');

        if(this.run.ui_jsm && (this.step.trajectories || this.step.trajectories_compressed)){
            let trajectories_path = this.step.trajectories_compressed || this.step.trajectories
            if(options["verbose"]){
                const url = new URL("./play_trajectories.html", window.location.href)
                url.searchParams.append("experiments", run.fs.base_path)
                url.searchParams.append("trajectories", trajectories_path)
                if(!run.ui_jsm){
                    throw `No ui_jsm found in ${this.run.config.path}"`
                }
                url.searchParams.append("ui", run.ui_jsm)
                const link = document.createElement('a');
                link.href = url.href;
                link.innerText = "Play isolated"
                this.content.appendChild(link);
            }

            const play_button = document.createElement('button');
            play_button.innerHTML = "Play Trajectories";
            this.content.appendChild(play_button);
            this.trajectory_player_container = document.createElement('div');
            this.trajectory_player_container.classList.add("explorer-trajectory-player-container")
            this.trajectory_player_container.style.display = "none";
            this.content.appendChild(this.trajectory_player_container);
            play_button.addEventListener('click', () => {
                const trajectory_player = new TrajectoryPlayer(run.ui_jsm);
                this.trajectory_player_container.appendChild(trajectory_player.getCanvas());
                this.trajectory_player_container.style.display = "block";
                trajectory_player.playTrajectories(trajectories_path);
            })
        }

        const step_spoiler = new Spoiler(parent, this.step.step, true, () => {
        }, () => {
            this.trajectory_player_container.innerHTML = "";
            this.trajectory_player_container.style.display = "none";
        });
        step_spoiler.setContent(this.content)
    }
}
export class ExplorerRun{
    constructor(parent, run, options){
        this.config = null
        this.container = document.createElement('div');
        this.container.classList.add("run-container");
        parent.setContent(this.container);

        this.steps_spoiler = new Spoiler(this.container, "Steps", false, (spoiler) => {
            for(const step of Object.keys(run.steps).sort()) {
                // const step_spoiler = new Spoiler(this.steps_spoiler, this.steps[step_id], );
                new ExplorerStep(spoiler, run, run.steps[step], options);
            }
        });
    }
}

export class Explorer{
    constructor(index, options){
        this.container = document.createElement('div');
        this.loading_text = document.createElement('div');
        this.loading_text.style.display = "block";
        this.container.appendChild(this.loading_text);
        index.refresh().then(() => {
            this.experiments = index.run_hierarchy;
            this.loading_text.style.display = "none";
            const experiment_list = document.createElement('ul');
            experiment_list.classList.add("experiment-list");
            this.container.appendChild(experiment_list);
            for (const experiment of Object.keys(this.experiments).sort().reverse()){
                new Spoiler(experiment_list, experiment, false, (experiment_spoiler) => {
                    for (const population of Object.keys(this.experiments[experiment]).sort()) {
                        const population_spoiler = new Spoiler(experiment_spoiler, population, false);
                        for (const config of Object.keys(this.experiments[experiment][population]).sort()) {
                            const config_spoiler = new Spoiler(population_spoiler, config, false);
                            for (const seed of Object.keys(this.experiments[experiment][population][config]).sort()) {
                                const run = this.experiments[experiment][population][config][seed];
                                const seed_label = run.config._source ? `${seed} [${run.config._source}]` : seed;
                                const seed_spoiler = new Spoiler(config_spoiler, seed_label, true);
                                new ExplorerRun(seed_spoiler, run, options);
                            }
                        }
                    }
                });
            }
        })
    }
    getContainer(){
        return this.container;
    }
}