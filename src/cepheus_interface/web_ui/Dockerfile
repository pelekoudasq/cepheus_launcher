FROM node:12.19.0-alpine3.10

WORKDIR /app

ENV PATH /app/node_modules/.bin:$PATH

COPY package-lock.json ./
COPY package.json ./
RUN npm install

COPY ./ ./

CMD ["npm", "start"]
