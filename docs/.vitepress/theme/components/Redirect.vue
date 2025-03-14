<template>
</template>


<script>
    /** Redirects to the given 'to' url, which is relative to the current location. Originates from https://github.com/vuejs/vuepress/issues/239#issuecomment-632567115 */
    export default {
        name: 'Redirect',
        props: {
            to: {
                type: String,
                required: true
            }
        },
        beforeMount() {
            document.location.replace(this.to);
        }
    }
</script>