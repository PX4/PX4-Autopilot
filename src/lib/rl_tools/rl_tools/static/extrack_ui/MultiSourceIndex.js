import { Index } from "./Index.js";

export class MultiSourceIndex {
    constructor(sources) {
        // sources is an array of {fs, label} objects
        this.sources = sources;
        this.indexes = sources.map(source => new Index(source.fs));
        this.run_list = null;
        this.run_hierarchy = null;
    }

    async refresh() {
        // Refresh all source indexes in parallel, with error handling
        const refresh_results = await Promise.allSettled(
            this.indexes.map(index => index.refresh())
        );

        // Merge all run lists from successful sources
        this.run_list = [];
        for (let i = 0; i < this.indexes.length; i++) {
            if (refresh_results[i].status === 'fulfilled') {
                const index = this.indexes[i];
                const source_label = this.sources[i].label;
                
                // Add source label to each run's config
                for (const run of index.run_list) {
                    run.config._source = source_label;
                    this.run_list.push(run);
                }
            } else {
                console.warn(`Failed to load runs from source "${this.sources[i].label}":`, refresh_results[i].reason);
            }
        }

        // Sort by experiment key (reverse chronological)
        this.run_list.sort((a, b) => {
            const experiment_compare = b.config.experiment.localeCompare(a.config.experiment);
            if (experiment_compare !== 0) return experiment_compare;
            
            // If experiments are the same, sort by commit
            const commit_compare = b.config.commit.localeCompare(a.config.commit);
            if (commit_compare !== 0) return commit_compare;
            
            // If commits are the same, sort by population values
            const population_compare = b.config.population_values.localeCompare(a.config.population_values);
            if (population_compare !== 0) return population_compare;
            
            // If everything else is the same, prefer curated over ci
            if (a.config._source !== b.config._source) {
                return a.config._source === 'curated' ? -1 : 1;
            }
            
            // Finally sort by seed
            return b.config.seed.localeCompare(a.config.seed);
        });

        // Build merged hierarchy
        this.run_hierarchy = {};
        for (const run of this.run_list) {
            let current = this.run_hierarchy;
            for (const key of ["experiment", "commit_population_variates", "population_values"]) {
                let value;
                if (key === "commit_population_variates") {
                    value = run.config["commit"] + "_" + run.config["name"] + "_" + run.config["population_variates"];
                } else {
                    value = run.config[key];
                }
                if (!(value in current)) {
                    current[value] = {};
                }
                current = current[value];
            }
            current[run.config["seed"]] = run;
        }

        return this;
    }
}

