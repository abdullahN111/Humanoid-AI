import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'ROS 2 as a Robotic Nervous System',
  tagline: 'Documentation for Physical AI and Humanoid Robotics',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://your-docusaurus-site.example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'facebook', // Usually your GitHub org/user name.
  projectName: 'docusaurus', // Usually your repo name.

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-username/ros2-nervous-system-docs/tree/main/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themes: [
    // ... other themes
  ],

  plugins: [
    // ... other plugins
    // Plugin to inject environment variables
    async function myPlugin(context, options) {
      const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || process.env.NEXT_PUBLIC_API_BASE_URL || 'http://localhost:8000';
      return {
        name: 'docusaurus-plugin-env-variables',
        injectHtmlTags() {
          return {
            postBodyTags: [
              `<script>
                window.ENV = {
                  API_BASE_URL: '${API_BASE_URL}'
                };
              </script>`,
            ],
          };
        },
      };
    },
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'ROS 2 Nervous System',
      logo: {
        alt: 'ROS 2 Logo',
        src: 'img/ros2-logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Module 1',
        },
        {
          type: 'doc',
          position: 'left',
          label: 'Module 2',
          docId: 'module-2/chapter-1',
        },
        {
          type: 'doc',
          position: 'left',
          label: 'Module 3',
          docId: 'module-3/intro',
        },
        {
          type: 'doc',
          position: 'left',
          label: 'Module 4',
          docId: 'module-4/chapter-1',
        },
        {to: '/blog', label: 'Blog', position: 'left'},
        {
          href: 'https://github.com/ros2/docs',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Documentation',
          items: [
            {
              label: 'Module 1: ROS 2 Fundamentals',
              to: '/docs/module-1/intro',
            },
            {
              label: 'Module 2: Digital Twin Simulation',
              to: '/docs/module-2/chapter-1',
            },
            {
              label: 'Module 3: The AI-Robot Brain',
              to: '/docs/module-3/intro',
            },
            {
              label: 'Module 4: Vision-Language-Action (VLA) Integration',
              to: '/docs/module-4/chapter-1',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'ROS Answers',
              href: 'https://answers.ros.org/',
            },
            {
              label: 'ROS Discourse',
              href: 'https://discourse.ros.org/',
            },
            {
              label: 'Robotics Stack Exchange',
              href: 'https://robotics.stackexchange.com/',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'ROS 2 Documentation',
              href: 'https://docs.ros.org/en/rolling/',
            },
            {
              label: 'Gazebo Simulation',
              href: 'https://gazebosim.org/',
            },
            {
              label: 'Unity Robotics',
              href: 'https://unity.com/solutions/industries/robotics',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/ros2/docs',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} ROS 2 Documentation Project. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
