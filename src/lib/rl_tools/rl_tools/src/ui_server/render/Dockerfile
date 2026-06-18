FROM ghcr.io/puppeteer/puppeteer:latest
USER root
RUN mkdir render
WORKDIR render
COPY package.json .
COPY package-lock.json .
COPY download_dependencies.sh .
RUN ./download_dependencies.sh
COPY puppeteer.* .
RUN chown -R pptruser:pptruser .
USER pptruser
RUN npm install
CMD ["node", "puppeteer.js"]