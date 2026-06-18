import {make_chart} from "./ZooLearningCurves.js"
import {group_by} from "./Index.js"



function aggregate(evaluation_data){
    const steps = []
    const returns_mean = []
    const returns_std = []
    let first_run = true
    for(const run of evaluation_data){
        if(first_run){
            steps.push(...run.data.map(entry => entry.step))
            first_run = false
            returns_mean.push(...run.data.map(entry => entry.returns_mean))
            returns_std.push(...run.data.map(entry => entry.returns_mean * entry.returns_mean))
        }
        else{
            console.assert(steps.length === run.data.length, "Steps are not the same")
            console.assert(JSON.stringify(steps) == JSON.stringify(run.data.map(entry => entry.step)), "Steps are not the same")
            for(const i in run.data){
                returns_mean[i] += run.data[i].returns_mean
                returns_std[i] += run.data[i].returns_mean * run.data[i].returns_mean
            }
        }
    }
    returns_mean.forEach((value, index) => {
        returns_mean[index] = value / evaluation_data.length
        returns_std[index] = Math.sqrt(returns_std[index] / evaluation_data.length - returns_mean[index] * returns_mean[index])
    })
    return steps.map((step, index) => {
        return {
            step: step,
            returns_mean: returns_mean[index],
            returns_std: returns_std[index]
        }
    })
}

export class Zoo{
    constructor(selectedRuns = []){
        this.container = document.createElement("div")
        this.container.classList.add("zoo-container")
        this.success = this.processRuns(selectedRuns);
    }
    
    async processRuns(selectedRunsData) {
        // Clear container
        this.container.innerHTML = '';
        
        if (!selectedRunsData || !Array.isArray(selectedRunsData) || selectedRunsData.length === 0) {
            this.container.innerHTML = '<p style="padding: 20px;">No runs selected. Please select runs from the table above.</p>';
            return false;
        }
        
        // Extract actual run objects and filter for those with return data
        const run_list = selectedRunsData.map(rd => rd.run).filter((run) => run.return)
        
        if(run_list.length === 0){
            this.container.innerHTML = '<p style="padding: 20px;">No runs with return data available.</p>';
            return false
        }
        
        // Group by algorithm and environment (not population_values which can vary)
        const run_list_grouped_by_algo_env = {};
        for (const run of run_list) {
            const algo = run.config.population.algorithm || 'unknown';
            const env = run.config.population.environment || 'unknown';
            const key = `${algo}_${env}`;
            
            if (!run_list_grouped_by_algo_env[key]) {
                run_list_grouped_by_algo_env[key] = {
                    items: [],
                    algorithm: algo,
                    environment: env
                };
            }
            run_list_grouped_by_algo_env[key].items.push(run);
        }
        
        const run_list_grouped_truncated = Object.fromEntries(
            Object.entries(run_list_grouped_by_algo_env).map(([key, group]) => {
                const population_experiments = group_by(group.items, ["experiment", "commit"])
                const experiment_keys = Object.keys(population_experiments).sort().reverse()
                return [key, Object.fromEntries(experiment_keys.map(expKey => [expKey, population_experiments[expKey]]))];
            })
        )
        const all_data = Object.fromEntries(await Promise.all(Object.keys(run_list_grouped_truncated).map(async populationKey => {
            const group = run_list_grouped_by_algo_env[populationKey];
            const population_data = Object.fromEntries(await Promise.all(Object.entries(run_list_grouped_truncated[populationKey]).map(async ([experiment, experiment_runs]) => {
                return [experiment, await Promise.all(experiment_runs.items.map(async (run) => {
                    let return_data = null
                    try{
                        return_data = await (await fetch(run.return)).text()
                    }
                    catch(e){
                        console.error(`Failed to fetch return data from ${run.return}`)
                        // throw new Error(`Failed to fetch return data from ${run.return}`)
                    }
                    let data = null;
                    try{
                        data = JSON.parse(return_data)
                    }
                    catch(e){
                        console.error(`Failed to parse return data from ${run.return}`)
                        // throw new Error(`Failed to parse return data from ${run.return}`)
                    }
                    return {
                        config: run.config,
                        label: run.config["seed"],
                        data: data
                    }
                }))]
            })))
            return [populationKey, {
                algorithm: group.algorithm,
                environment: group.environment,
                data: population_data
            }]
        })))


        for(const populationKey in all_data){
            const populationInfo = all_data[populationKey];
            const algorithm = populationInfo.algorithm;
            const environment = populationInfo.environment;
            
            const aggregated_data = Object.entries(populationInfo.data).map(([experiment, experiment_data]) => {
                return {
                    label: experiment,
                    data: aggregate(experiment_data)
                }
            })
            
            const header = document.createElement("div")
            header.style.fontSize = "1.5em"
            header.innerHTML = `Algorithm: <b>${algorithm}</b>, Environment: <b>${environment}</b>`
            this.container.appendChild(header)
            const chart = make_chart(aggregated_data)
            this.container.appendChild(chart)
        }
        return true
    }
    
    updateRuns(selectedRunsData) {
        this.success = this.processRuns(selectedRunsData);
        return this.success;
    }
    getContainer(){
        return this.container
    }

}