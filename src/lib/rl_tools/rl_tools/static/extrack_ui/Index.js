class Step{
    constructor(run, step, node){
        this.run = run;
        this.step = step;
        this.node = node;
        this.checkpoint_code = "checkpoint.h" in node.children ? node.children["checkpoint.h"] : null
        this.checkpoint_hdf5 = "checkpoint.h5" in node.children ? node.children["checkpoint.h5"] : null
        this.trajectories = "trajectories.json" in node.children ? node.children["trajectories.json"] : null
        this.trajectories_compressed = "trajectories.json.gz" in node.children ? node.children["trajectories.json.gz"] : null
    }

}

class Run{
    constructor(fs, node, config){
        this.fs = fs;
        this.config = config;
        this.node = node;
        this.load(this.node)
    }

    load(node){
        this.ui_js = "ui.js" in node.children ? node.children["ui.js"] : null
        this.ui_jsm = "ui.esm.js" in node.children ? node.children["ui.esm.js"] : null
        this.description = "description.txt" in node.children ? node.children["description.txt"] : null
        this.return = "return.json" in node.children ? node.children["return.json"] : null
        this.steps = {}
        if("steps" in node.children){
            for(const step_id in node.children["steps"].children){
                const step_node = node.children["steps"].children[step_id]
                this.steps[step_id] = new Step(this, step_id, step_node)
            }
        }
    }

    async refresh(){
        this.node = await this.fs.refresh(this.node)
        this.load(this.node)
    }

}

export function group_by(array, keys){
    const groups = {}
    for(const item of array){
        const master_key = keys.map(key => item.config[key]).join("_")
        if(!(master_key in groups)){
            groups[master_key] = {
                keys: Object.fromEntries(keys.map(key => [key, item[key]])),
                items: []
            }
        }
        groups[master_key].items.push(item)
    }
    return groups
}
export class Index{
    constructor(fs){
        this.fs = fs;
        this.run_list = null // this is always sorted lexicographically
        this.run_hierarchy = null
    }
    async refresh(){
        const tree = await this.fs.loadTree()
        const run_list = []
        for(const experiment_key of Object.keys(tree.children).sort().reverse()){
            if(experiment_key === "index.txt" || experiment_key === "index.txt.tmp"){
                continue
            }
            const run_config = {
                "experiment": experiment_key
            }
            const experiment = tree.children[experiment_key]
            for(const commit_population_key of Object.keys(experiment.children).sort()){
                const commit = commit_population_key.split("_")[0]
                const name = commit_population_key.split("_")[1]
                const population_variates = commit_population_key.split("_").slice(2)
                const population_config = {...run_config, "commit": commit, "name": name, "population_variates": population_variates.join("_")}
                const commit_population = experiment.children[commit_population_key]
                for(const config_key of Object.keys(commit_population.children).sort()){
                    const population_values = config_key.split("_")
                    const config_config = {...population_config, "population_values": population_values.join("_"), "population": Object.fromEntries(population_variates.map((key, index) => [key, population_values[index]]))}
                    const config = commit_population.children[config_key]
                    for(const seed_key of Object.keys(config.children).sort()){
                        const seed_config = {...config_config, "seed": seed_key}
                        const run = new Run(this.fs, config.children[seed_key], seed_config)
                        run_list.push(run)
                    }
                }
            }
        }
        const run_hierarchy = {}
        for(const run of run_list){
            let current = run_hierarchy
            for(const key of ["experiment", "commit_population_variates", "population_values"]){
                let value;
                if(key === "commit_population_variates"){
                    value = run.config["commit"] + "_" + run.config["name"] + "_" + run.config["population_variates"]
                }
                else{
                    value = run.config[key]
                }
                if(!(value in current)){
                    current[value] = {}
                }
                current = current[value]
            }
            current[run.config["seed"]] = run
        }
        this.run_list = run_list
        this.run_hierarchy = run_hierarchy
        return this
    }
}