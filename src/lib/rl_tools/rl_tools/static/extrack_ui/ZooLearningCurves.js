import {Chart, LineController, ScatterController, LineElement, PointElement, LinearScale, CategoryScale, Title, Tooltip, Filler, Legend} from "chart.js"
Chart.register(LineController, ScatterController, LineElement, PointElement, LinearScale, CategoryScale, Title, Tooltip, Filler, Legend);

function hexToRgba(hex, alpha = 1.0) {
    hex = hex.replace(/^#/, '');

    let r = parseInt(hex.substring(0, 2), 16);
    let g = parseInt(hex.substring(2, 4), 16);
    let b = parseInt(hex.substring(4, 6), 16);

    return `rgba(${r}, ${g}, ${b}, ${alpha})`;
}

export function make_chart(data) {
    const container = document.createElement("div");
    container.classList.add("zoo-chart-container");
    const canvas = document.createElement("canvas");
    canvas.classList.add("zoo-chart");
    container.appendChild(canvas);
    const ctx = canvas.getContext('2d');

    const steps = Array.from(new Set(data.reduce((a, c) => [...a, ...c.data.map(step => step.step)], []))).sort((a, b) => a - b);
    for (const d of data) {
        d.mean_returns = steps.map(step => {
            const entry = d.data.find(d => d.step === step);
            return entry ? entry.returns_mean : null;
        });
        d.std_returns = steps.map(step => {
            const entry = d.data.find(d => d.step === step);
            return entry ? entry.returns_std : null;
        });
        d.mean_plus_std = d.mean_returns.map((mean, index) => {
            const std = d.std_returns[index];
            return mean !== null && std !== null ? mean + std : null;
        });

        d.mean_minus_std = d.mean_returns.map((mean, index) => {
            const std = d.std_returns[index];
            return mean !== null && std !== null ? mean - std : null;
        });
    }

    let color_palette = [
        "#e41a1c", // red
        "#377eb8", // blue
        "#4daf4a", // green
        "#984ea3", // purple
        "#ff7f00", // orange
        "#ffff33", // yellow
        "#a65628", // brown
        "#f781bf", // pink
        "#999999"  // gray
    ];
    let index = 0;

    let datasets = [];
    for (const d of data) {
        const color_hex = color_palette[index]
        index = (index + 1) % color_palette.length;
        const color = hexToRgba(color_hex);
        const color_ribbon = hexToRgba(color_hex, 0.2);
        datasets.push({
            label: d.label,
            data: d.mean_returns,
            borderColor: color,
            backgroundColor: color_ribbon,
            yAxisID: 'y',
            fill: false,
            spanGaps: true
        });
        datasets.push({
            label: `${d.label} +Standard Deviation`,
            data: d.mean_plus_std,
            borderColor: 'rgba(0, 0, 0, 0)',
            backgroundColor: color_ribbon,
            yAxisID: 'y',
            fill: '+1',
            pointRadius: 0,
            pointHoverRadius: 0,
            spanGaps: true
        });
        datasets.push({
            label: `${d.label} -Standard Deviation`,
            data: d.mean_minus_std,
            borderColor: 'rgba(0, 0, 0, 0)',
            backgroundColor: color_ribbon,
            yAxisID: 'y',
            fill: false,
            pointRadius: 0,
            pointHoverRadius: 0,
            spanGaps: true
        });
    }


    const myChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: steps,
            datasets: datasets
        },
        options: {
            animation: false,
            scales: {
                y: {
                    type: 'linear',
                    position: 'left',
                    ticks: {
                        beginAtZero: true
                    },
                    title: {
                        display: true,
                        text: 'Return'
                    }
                },
                x: {
                    title: {
                        display: true,
                        text: 'Step'
                    }
                }
            },
            plugins: {
                legend: {
                    labels: {
                        filter: function(legendItem, chartData) {
                            return !legendItem.text.endsWith('Standard Deviation');
                        }
                    }
                }
            }
        }
    });
    return container;
}
