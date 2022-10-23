import { defaultTheme, defineUserConfig } from 'vuepress'

export default defineUserConfig({
  title: "Simply 3d printer",
  description: "Find out how 3d printer works by writing a firmware",
  base: `/simple-3d-printer/`,
  head: [
    [
      'link', { rel: 'icon', href: '/simple-3d-printer/favicon.ico'}
    ]
  ],
  theme: defaultTheme({
    logo: 'logo.svg',
    logoDark: 'logo-dark.svg',
    locales: {
      '/': {
        selectLanguageName: 'English',
      },
      '/zh/': {
        selectLanguageName: '简体中文',
      },
    },
  }),
  locales: {
    '/': {
      lang: 'en-US',
      title: 'Simply 3d printer',
      description: 'Find out how 3d printer works by writing a firmware',
    },
    '/zh/': {
      lang: 'zh-CN',
      title: '简单3d打印机',
      description: '通过编写一个打印机固件学习3d打印机原理',
    },
  }
})
