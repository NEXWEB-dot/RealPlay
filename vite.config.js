import { defineConfig } from 'vite';
import basicSsl from '@vitejs/plugin-basic-ssl';

/**
 * REALPLAY Vite config.
 *
 * - MPA: two pages are built, the PC game (index.html) and the phone
 *   controller (controller/index.html). appType 'mpa' disables the SPA
 *   fallback so /controller/* can never accidentally serve the PC app.
 * - /ws is proxied to the Node game server so the PC app and the phones
 *   talk to a single origin (important for the QR code).
 * - Set RP_HTTPS=1 (npm run dev:https) to enable a self-signed HTTPS dev
 *   server. Phone browsers only expose motion sensors in secure contexts,
 *   so LAN testing over plain HTTP will show no sensor data on iOS/Android.
 */
export default defineConfig(({ mode }) => ({
  appType: 'mpa',
  plugins: process.env.RP_HTTPS ? [basicSsl()] : [],
  server: {
    host: true, // listen on all interfaces so phones on the LAN can connect
    port: 5173,
    proxy: {
      '/ws': {
        target: 'http://127.0.0.1:8081',
        ws: true,
      },
    },
  },
  build: {
    rollupOptions: {
      input: {
        main: 'index.html',
        controller: 'controller/index.html',
        markers: 'markers.html',
      },
    },
    chunkSizeWarningLimit: 1500,
  },
}));
