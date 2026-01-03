// @ts-check
// `@docusaurus/types` added to `devDependencies` in `package.json`
const { themes } = require('prism-react-renderer');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI and Humanoid Book',
  tagline: 'Comprehensive Educational System for Physical AI & Humanoid Robotics',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-project-name.vercel.app',  // Update to your Vercel deployment URL
  // Set the /<baseUrl>/ pathname under which your site is served
  // For Vercel deployment, typically use root path
  baseUrl: '/',

  // GitHub pages deployment config (remove or comment out for Vercel)
  // organizationName: 'Musabbihakamal', // GitHub username for GitHub Pages
  // projectName: 'Physical-AI-and-Humanoid-Book', // GitHub repo name for GitHub Pages

  onBrokenLinks: 'throw',
  markdown: {
    mermaid: true,
    format: 'detect',
  },

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
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Musabbihakamal/Physical-AI-and-Humanoid-Book/edit/main/frontend/docs/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI and Humanoid Book',
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Docs',
          },
          {
            href: '/auth/signin',
            label: 'Sign In',
            position: 'right',
          },
          {
            href: '/auth/signup',
            label: 'Sign Up',
            position: 'right',
          },
          {
            href: 'https://github.com/Musabbihakamal/Physical-AI-and-Humanoid-Book',
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
                label: 'Tutorials',
                to: '/docs/category/tutorials',
              },
              {
                label: 'API Reference',
                href: 'https://github.com/Musabbihakamal/Physical-AI-and-Humanoid-Book#api-documentation', // Link to README section
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/Musabbihakamal/Physical-AI-and-Humanoid-Book',
              },
              {
                label: 'Issues',
                href: 'https://github.com/Musabbihakamal/Physical-AI-and-Humanoid-Book/issues',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/Musabbihakamal/Physical-AI-and-Humanoid-Book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI and Humanoid Book Project. Built with Docusaurus.`,
      },
      prism: {
        theme: themes.github,
        darkTheme: themes.dracula,
      },
      // Enable mermaid
      mermaid: {
        options: {
          maxTextSize: 50000,
        },
      },
    }),
  };

module.exports = config;