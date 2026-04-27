import { defineConfig } from 'vite';

export default defineConfig({
  server: {
    port: 3000,
    proxy: {
      '/api': {
        target: 'http://localhost:8765',
        changeOrigin: true,
        rewrite: (path) => path.replace(/^\/api/, ''),
      },
      '/parametric': {
        target: 'http://localhost:8765',
        changeOrigin: true,
      },
    },
  },
  publicDir: '../data',
});
