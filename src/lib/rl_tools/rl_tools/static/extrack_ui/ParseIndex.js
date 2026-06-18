export function parseIndex(index){
    const regex_experiment = /^\.\/([^\/]+)/
    const regex_commit_hash = new RegExp(regex_experiment.source + /\/([^_]+)/.source)
    const regex_config_population = new RegExp(regex_commit_hash.source + /_([^\/]+)/.source)
    const regex_config = new RegExp(regex_config_population.source + /\/([^\/]+)/.source)
    const regex_seed = new RegExp(regex_config.source + /\/(\d+)/.source)
    const regex_ui_js = new RegExp(regex_seed.source + /\/ui\.js/.source)
    const regex_ui_jsm = new RegExp(regex_seed.source + /\/ui\.esm\.js/.source)
    const regex_step = new RegExp(regex_seed.source + /\/steps\/(\d+)/.source)
    const regex_checkpoint_code = new RegExp(regex_step.source + /\/checkpoint\.h/.source)
    const regex_checkpoint_hdf5 = new RegExp(regex_step.source + /\/checkpoint\.h5/.source)
    const trajectories = new RegExp(regex_step.source + /\/trajectories\.json/.source)
    const trajectories_compressed = new RegExp(regex_step.source + /\/trajectories\.json.gz/.source)
    const lines = index.split("\n");
    const experiments_list = [];
    for (const line of lines){
        const experiment = {
            "path": line
        };
        const experiment_match = line.match(regex_experiment);
        if (experiment_match){
            if(experiment_match[1] !== "index.txt"){
                experiment["experiment"] = experiment_match[1];
            }
        }
        else{
            continue;
        }
        const commit_hash_match = line.match(regex_commit_hash);
        if (commit_hash_match){
            experiment["commit_hash"] = commit_hash_match[2];
        }
        else{
            continue;
        }
        const config_population_match = line.match(regex_config_population);
        if (config_population_match){
            experiment["config_population"] = config_population_match[3];
        }
        else{
            continue;
        }
        const config_match = line.match(regex_config);
        if (config_match){
            experiment["config"] = config_match[4];
        }
        else{
            continue;
        }
        const seed_match = line.match(regex_seed);
        if (seed_match){
            experiment["seed"] = seed_match[5];
        }
        else{
            continue;
        }
        if (line.match(regex_ui_js)){
            experiment["ui_js"] = true;
        }
        if (line.match(regex_ui_jsm)){
            experiment["ui_jsm"] = true;
        }
        const step_match = line.match(regex_step);
        if (step_match){
            experiment["step"] = step_match[6];
        }
        if (line.match(regex_checkpoint_code)){
            experiment["checkpoint_code"] = true;
        }
        if (line.match(regex_checkpoint_hdf5)){
            experiment["checkpoint_hdf5"] = true;
        }
        if (line.match(trajectories)){
            experiment["trajectories"] = true;
        }
        if (line.match(trajectories_compressed)) {
            experiment["trajectories_compressed"] = true;
        }
        experiments_list.push(experiment);
    }
    const experiments = {}
    for(const experiment of experiments_list){
        if (experiment.experiment in experiments){
            experiments[experiment.experiment].push(experiment);
        } else {
            experiments[experiment.experiment] = [experiment];
        }
    }
    const experiments_population = {}
    for(const experiment in experiments){
        experiments_population[experiment] = {}
        for(const config of experiments[experiment]){
            const population = config.commit_hash + "_" + config.config_population;
            if (population in experiments_population[experiment]){
                experiments_population[experiment][population].push(config);
            } else {
                experiments_population[experiment][population] = [config];
            }
        }
    }
    const experiments_config = {}
    for(const experiment in experiments_population){
        experiments_config[experiment] = {}
        for(const population in experiments_population[experiment]){
            experiments_config[experiment][population] = {}
            for(const config of experiments_population[experiment][population]){
                if (config.config in experiments_config[experiment][population]){
                    experiments_config[experiment][population][config.config].push(config);
                } else {
                    experiments_config[experiment][population][config.config] = [config];
                }
            }
        }
    }
    const experiments_seed = {}
    for(const experiment in experiments_config){
        experiments_seed[experiment] = {}
        for(const population in experiments_config[experiment]){
            experiments_seed[experiment][population] = {}
            for(const config in experiments_config[experiment][population]){
                experiments_seed[experiment][population][config] = {}
                for(const seed of experiments_config[experiment][population][config]){
                    if (seed.seed in experiments_seed[experiment][population][config]){
                        experiments_seed[experiment][population][config][seed.seed].push(seed);
                    } else {
                        experiments_seed[experiment][population][config][seed.seed] = [seed];
                    }
                }
            }
        }
    }
    return experiments_seed;
}
