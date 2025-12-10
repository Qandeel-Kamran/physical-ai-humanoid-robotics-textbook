#!/bin/bash

# Script to deploy the Docusaurus textbook to GitHub Pages

# Create and setup the Docusaurus website
mkdir -p textbook-website/my-textbook-website
cd textbook-website/my-textbook-website

# Initialize npm and install Docusaurus
npm init -y

npm install @docusaurus/core @docusaurus/preset-classic

# Create the Docusaurus configuration
cat > docusaurus.config.js << 'EOF'
// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)
/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'A comprehensive textbook on Physical AI and Humanoid Robotics',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://ameen-alam.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/Physical-AI-Humanoid-Robotics-Textbook/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'ameen-alam', // Usually your GitHub org/user name.
  projectName: 'Physical-AI-Humanoid-Robotics-Textbook', // Usually your repo name.

  onBrokenLinks: 'warn',
  onBrokenMarkdownImages: 'warn', // This is not a valid option, will handle via other means

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
          sidebarPath: './sidebars.js',
          // Remove the edit URL since this is textbook content
          editUrl: undefined,
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      colorMode: {
        respectPrefersColorScheme: true,
      },
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Physical AI & Humanoid Robotics Textbook',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'textbookSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            href: 'https://github.com/ameen-alam/Physical-AI-Humanoid-Robotics-Textbook',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Textbook',
            items: [
              {
                label: 'Introduction',
                to: '/docs/chapter-01-introduction',
              },
              {
                label: 'All Chapters',
                to: '/docs/chapter-01-introduction',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'GitHub Repository',
                href: 'https://github.com/ameen-alam/Physical-AI-Humanoid-Robotics-Textbook',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'About the Author',
                href: 'https://github.com/ameen-alam',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;
EOF

# Create the sidebar configuration
cat > sidebars.js << 'EOF'
// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // Textbook sidebar with all chapters in proper order
  textbookSidebar: [
    {
      type: 'category',
      label: 'Physical AI & Humanoid Robotics Textbook',
      collapsed: false,
      items: [
        'chapter-01-introduction',
        'chapter-02-biomechanics',
        'chapter-03-sensorimotor',
        'chapter-04-cognitive-architecture',
        'chapter-05-actuation',
        'chapter-06-control-theory',
        'chapter-07-locomotion',
        'chapter-08-manipulation',
        'chapter-09-perception',
        'chapter-10-learning',
        'chapter-11-hri',
        'chapter-12-simulation',
        'chapter-13-hardware',
        'chapter-14-energy',
        'chapter-15-safety',
        'chapter-16-applications',
        'chapter-17-future-directions',
        'chapter-18-ethics',
        'chapter-18-conclusion',
        'textbook-summary'
      ],
    },
  ],
};

export default sidebars;
EOF

# Create the docs directory and copy textbook content
mkdir -p docs

# Copy the textbook chapters from the parent directory
cp -r ../../docs/* docs/

# Replace image references in markdown files with placeholders
for file in docs/*.md; do
  if [ -f "$file" ]; then
    sed -i 's/!\[.*\](\.\/assets\/.*\.png)/\[Image: Reference to diagram or illustration\]/g' "$file"
    sed -i 's/!\[.*\](\.\/assets\/.*\.jpg)/\[Image: Reference to diagram or illustration\]/g' "$file"
    sed -i 's/!\[.*\](\.\/assets\/.*\.gif)/\[Image: Reference to diagram or illustration\]/g' "$file"
  fi
done

# Create basic directory structure if it doesn't exist
mkdir -p src/css src/pages static/img

# Create a basic custom CSS file
cat > src/css/custom.css << 'EOF'
/**
 * Any CSS included here will be global. The classic template
 * bundles Infima by default. Infima is a CSS framework designed to
 * work well for content-centric websites.
 */

/* You can override the default Infima variables here. */
:root {
  --ifm-color-primary: #2e8555;
  --ifm-color-primary-dark: #29784c;
  --ifm-color-primary-darker: #277148;
  --ifm-color-primary-darkest: #205d3b;
  --ifm-color-primary-light: #33925d;
  --ifm-color-primary-lighter: #359962;
  --ifm-color-primary-lightest: #3cad6e;
  --ifm-code-font-size: 95%;
  --docusaurus-highlighted-code-line-bg: rgba(0, 0, 0, 0.1);
}

/* For readability concerns, you should choose a lighter palette in dark mode. */
[data-theme='dark'] {
  --ifm-color-primary: #25c2a0;
  --ifm-color-primary-dark: #21af90;
  --ifm-color-primary-darker: #1fa588;
  --ifm-color-primary-darkest: #1a8870;
  --ifm-color-primary-light: #29d5b0;
  --ifm-color-primary-lighter: #32d8b4;
  --ifm-color-primary-lightest: #4fddbf;
  --docusaurus-highlighted-code-line-bg: rgba(0, 0, 0, 0.3);
}
EOF

# Create a basic logo file (placeholder)
echo "Logo content" > static/img/logo.svg

# Build the site
echo "Building the Docusaurus site..."
npm run build

# The built site will be in the build/ directory
echo "Build complete! Files are in the build/ directory"
EOF