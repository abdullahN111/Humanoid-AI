// Docusaurus plugin for AI Question Interface
const path = require('path');

module.exports = function (context, options) {
  return {
    name: 'docusaurus-ai-question-plugin',
    getThemePath() {
      return path.resolve(__dirname, '../..', 'components');
    },
    configureWebpack(config, isServer, utils) {
      return {
        resolve: {
          alias: {
            '@ai-question-interface': path.resolve(__dirname, '../../components/AIQuestionInterface'),
          },
        },
      };
    },
  };
};